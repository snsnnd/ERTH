#include "erth_battle_v2.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Implement these with the selected board's HAL/driver. */
extern void transport_write(const uint8_t *data, size_t length);
extern void motors_stop(void);
extern void motors_drive(int left_pwm, int right_pwm, uint16_t lease_ms);
extern void motors_estop(void);
extern bool motors_clear_estop(void);
extern void controller_set_mode(const char *mode);
extern void controller_set_target(int32_t x10, int32_t y10);
extern void controller_start(void);
extern uint32_t uptime_ms(void);
/* Return an 8-character uppercase hex boot counter that changes on every reset. */
extern void board_get_boot_id(char output[9]);

static erth_v2_stream_t rx_stream;
static char session_id[ERTH_V2_SID_BYTES] = "0000000000000000";
static const char vehicle_id[] = "A"; /* Assigned by the event/platform wiring. */
static uint32_t tx_seq;
static bool estop_latched;
static bool match_running;
static bool bench_mode;
static char control_mode[] = "MANUAL";
static char car_state[] = "PARKED";

static void send_frame(const erth_v2_frame_t *frame) {
    uint8_t output[ERTH_V2_MAX_FRAME_BYTES];
    size_t output_length = 0u;
    if (erth_v2_encode(frame, output, sizeof(output), &output_length) == ERTH_V2_OK) {
        transport_write(output, output_length);
    }
}

static void send_ready(const char *status) {
    erth_v2_frame_t frame;
    if (erth_v2_make_ready(&frame, session_id, vehicle_id, erth_v2_seq_next(&tx_seq), status) == ERTH_V2_OK) {
        send_frame(&frame);
    }
}

static void send_state(void) {
    erth_v2_frame_t frame;
    char flags[3] = {'0', estop_latched ? '1' : '0', '\0'};
    if (erth_v2_frame_init(&frame, session_id, vehicle_id, "HUB", erth_v2_seq_next(&tx_seq), "STATE") != ERTH_V2_OK) return;
    if (erth_v2_add_field(&frame, car_state) != ERTH_V2_OK ||
        erth_v2_add_field(&frame, control_mode) != ERTH_V2_OK ||
        erth_v2_add_field(&frame, vehicle_id) != ERTH_V2_OK ||
        erth_v2_add_field(&frame, flags) != ERTH_V2_OK) return;
    send_frame(&frame);
}

static void send_heartbeat_pong(void) {
    erth_v2_frame_t frame;
    char uptime[ERTH_V2_FIELD_BYTES];
    int length = snprintf(uptime, sizeof(uptime), "%lu", (unsigned long)uptime_ms());
    if (length < 0 || (size_t)length >= sizeof(uptime)) return;
    if (erth_v2_frame_init(&frame, session_id, vehicle_id, "HUB", erth_v2_seq_next(&tx_seq), "HEARTBEAT") != ERTH_V2_OK) return;
    if (erth_v2_add_field(&frame, "PONG") != ERTH_V2_OK ||
        erth_v2_add_field(&frame, uptime) != ERTH_V2_OK ||
        erth_v2_add_field(&frame, car_state) != ERTH_V2_OK) return;
    send_frame(&frame);
}

static void send_request_result(const erth_v2_frame_t *request, bool accepted, const char *reason) {
    erth_v2_frame_t response;
    erth_v2_result_t result;
    if (accepted) {
        result = erth_v2_make_ack(&response, session_id, vehicle_id, "HUB", erth_v2_seq_next(&tx_seq), request->seq);
    } else {
        result = erth_v2_make_nack(&response, session_id, vehicle_id, "HUB", erth_v2_seq_next(&tx_seq),
                                   request->seq, reason);
    }
    if (result == ERTH_V2_OK) send_frame(&response);
}

static void on_v2_frame(const erth_v2_frame_t *frame, void *context) {
    (void)context;
    if (frame == NULL || strcmp(frame->dst, vehicle_id) != 0 || strcmp(frame->src, "HUB") != 0) {
        return;
    }

    if (strcmp(frame->type, "WELCOME") == 0) {
        memcpy(session_id, frame->sid, sizeof(session_id));
        tx_seq = 0u;
        bench_mode = strcmp(frame->fields[4], "BENCH") == 0;
        /* Check role/map/capabilities before reporting READY in production firmware. */
        send_ready("OK");
        send_state();
        return;
    }
    if (strcmp(frame->sid, session_id) != 0) {
        motors_stop();
        return;
    }

    if (strcmp(frame->type, "HEARTBEAT") == 0 && strcmp(frame->fields[0], "PING") == 0) {
        send_heartbeat_pong();
        return;
    }

    if (strcmp(frame->type, "CMD") == 0 && frame->field_count > 0u) {
        const char *command = frame->fields[0];
        bool needs_ack = strcmp(command, "DRIVE") != 0;
        bool accepted = true;
        const char *error_code = "UNSUPPORTED";
        if (strcmp(command, "STOP") == 0) {
            match_running = false;
            motors_stop();
            (void)strcpy(car_state, "PARKED");
        } else if (strcmp(command, "ESTOP") == 0) {
            match_running = false;
            motors_estop();
            estop_latched = true;
            (void)strcpy(car_state, "ESTOP");
        } else if (strcmp(command, "ARM") == 0) {
            if (!match_running && motors_clear_estop()) {
                estop_latched = false;
                (void)strcpy(car_state, "PARKED");
            } else {
                accepted = false;
                error_code = "BAD_STATE";
            }
        } else if (strcmp(command, "START") == 0) {
            if (bench_mode && !estop_latched) {
                match_running = true;
                controller_start();
                (void)strcpy(car_state, "MOVING");
            } else {
                accepted = false;
                error_code = "BAD_STATE";
            }
        } else if (strcmp(command, "DRIVE") == 0 && frame->field_count == 4u) {
            int left = (int)strtol(frame->fields[1], NULL, 10);
            int right = (int)strtol(frame->fields[2], NULL, 10);
            uint16_t lease = (uint16_t)strtoul(frame->fields[3], NULL, 10);
            if (!estop_latched && match_running) {
                motors_drive(left, right, lease);
                (void)strcpy(car_state, (left == 0 && right == 0) ? "STOPPED" : "MOVING");
            } else {
                motors_stop();
                (void)strcpy(car_state, "PARKED");
            }
        } else if (strcmp(command, "MODE") == 0 && frame->field_count == 2u) {
            if (!estop_latched) {
                (void)strcpy(control_mode, frame->fields[1]);
                controller_set_mode(control_mode);
            } else {
                accepted = false;
                error_code = "BAD_STATE";
            }
        } else if (strcmp(command, "TARGET") == 0 && frame->field_count == 3u) {
            if (!estop_latched) {
                controller_set_target((int32_t)strtol(frame->fields[1], NULL, 10),
                                      (int32_t)strtol(frame->fields[2], NULL, 10));
            } else {
                accepted = false;
                error_code = "BAD_STATE";
            }
        } else {
            accepted = false;
        }
        if (needs_ack) {
            send_request_result(frame, accepted, error_code);
            send_state();
        }
    } else if (strcmp(frame->type, "MATCH") == 0 && frame->field_count >= 1u) {
        const char *action = frame->fields[0];
        bool accepted = true;
        if (strcmp(action, "START") == 0 || strcmp(action, "RESUME") == 0) {
            if (!estop_latched && !bench_mode) {
                match_running = true;
                controller_start();
                (void)strcpy(car_state, "MOVING");
            } else {
                accepted = false;
            }
        } else if (strcmp(action, "PAUSE") == 0 || strcmp(action, "ABORT") == 0 ||
                   strcmp(action, "FINISH") == 0) {
            match_running = false;
            motors_stop();
            (void)strcpy(car_state, "PARKED");
        } else if (strcmp(action, "PREPARE") == 0) {
            match_running = false;
            motors_stop();
            (void)strcpy(car_state, "PARKED");
        }
        send_request_result(frame, accepted, "BAD_STATE");
        send_state();
    }
}

void erth_participant_startup(void) {
    erth_v2_frame_t hello;
    char boot_id[9];
    erth_v2_stream_init(&rx_stream);
    motors_stop();
    board_get_boot_id(boot_id);
    boot_id[8] = '\0';
    if (erth_v2_make_hello(&hello, vehicle_id, "2.0.0", boot_id,
                           "DRIVE|POSE|CLAIM", 8u) == ERTH_V2_OK) {
        send_frame(&hello);
    }
}

void erth_participant_on_transport_bytes(const uint8_t *data, size_t length) {
    erth_v2_stream_feed(&rx_stream, data, length, on_v2_frame, NULL);
}
