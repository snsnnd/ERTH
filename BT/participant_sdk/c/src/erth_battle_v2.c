#include "erth_battle_v2.h"

#include <stdio.h>
#include <string.h>

#define ERTH_V2_MAX_TOKENS (6u + ERTH_V2_MAX_FIELDS)

static bool is_hex_upper(char c) {
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F');
}

static bool is_token_char(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
           (c >= '0' && c <= '9') || c == '_' || c == '.' ||
           c == '|' || c == ':' || c == '-';
}

static bool copy_text(char *destination, size_t capacity, const char *source) {
    size_t length;
    if (destination == NULL || source == NULL || capacity == 0u) {
        return false;
    }
    length = strlen(source);
    if (length == 0u || length >= capacity) {
        return false;
    }
    memcpy(destination, source, length + 1u);
    return true;
}

static bool valid_token(const char *value) {
    size_t i;
    if (value == NULL || value[0] == '\0') {
        return false;
    }
    for (i = 0u; value[i] != '\0'; ++i) {
        if (!is_token_char(value[i])) {
            return false;
        }
    }
    return true;
}

static bool valid_capabilities(const char *value) {
    const char *start;
    const char *cursor;
    if (value == NULL || value[0] == '\0') return false;
    if (strcmp(value, "-") == 0) return true;
    start = value;
    for (cursor = value; ; ++cursor) {
        if (*cursor == '|' || *cursor == '\0') {
            size_t length = (size_t)(cursor - start);
            size_t i;
            if (length == 0u) return false;
            for (i = 0u; i < length; ++i) {
                char c = start[i];
                if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                      (c >= '0' && c <= '9') || c == '_' || c == '.' || c == ':' || c == '-')) {
                    return false;
                }
            }
            if (*cursor == '\0') return true;
            start = cursor + 1;
        }
    }
}

static bool valid_hex(const char *value, size_t expected_length) {
    size_t i;
    if (value == NULL || strlen(value) != expected_length) {
        return false;
    }
    for (i = 0u; i < expected_length; ++i) {
        if (!is_hex_upper(value[i])) {
            return false;
        }
    }
    return true;
}

static bool valid_id(const char *value) {
    return value != NULL &&
           (strcmp(value, "HUB") == 0 || strcmp(value, "A") == 0 || strcmp(value, "B") == 0);
}

static bool parse_integer(const char *value, int64_t minimum, int64_t maximum) {
    int64_t number = 0;
    size_t i = 0u;
    bool negative = false;
    if (value == NULL || value[0] == '\0') {
        return false;
    }
    if (value[i] == '-') {
        negative = true;
        ++i;
        if (value[i] == '\0') {
            return false;
        }
    }
    if (value[i] == '0' && value[i + 1u] != '\0') {
        return false;
    }
    for (; value[i] != '\0'; ++i) {
        int digit;
        if (value[i] < '0' || value[i] > '9') {
            return false;
        }
        digit = value[i] - '0';
        if (number > (INT64_MAX - digit) / 10) {
            return false;
        }
        number = number * 10 + digit;
    }
    if (negative) {
        number = -number;
    }
    return number >= minimum && number <= maximum;
}

static bool field_count_is(const erth_v2_frame_t *frame, uint8_t count) {
    return frame->field_count == count;
}

static bool validate_schema(const erth_v2_frame_t *frame) {
    const char *f0;
    if (frame == NULL) {
        return false;
    }
    f0 = frame->field_count > 0u ? frame->fields[0] : "";

    if (strcmp(frame->type, "HELLO") == 0) {
        return field_count_is(frame, 4u) && valid_token(frame->fields[0]) &&
               valid_hex(frame->fields[1], 8u) && valid_capabilities(frame->fields[2]) &&
               parse_integer(frame->fields[3], 0, 255);
    }
    if (strcmp(frame->type, "WELCOME") == 0) {
        return field_count_is(frame, 5u) && valid_token(frame->fields[0]) &&
               (strcmp(frame->fields[1], "NONE") == 0 || valid_hex(frame->fields[1], 8u)) &&
               parse_integer(frame->fields[2], 1, 60000) &&
               parse_integer(frame->fields[3], 1, 400) &&
               (strcmp(frame->fields[4], "OFFICIAL") == 0 || strcmp(frame->fields[4], "BENCH") == 0);
    }
    if (strcmp(frame->type, "READY") == 0) {
        return field_count_is(frame, 1u) &&
               (strcmp(f0, "OK") == 0 ||
                (strncmp(f0, "NOT_READY_", 10u) == 0 && f0[10] != '\0' && valid_token(f0)));
    }
    if (strcmp(frame->type, "MATCH") == 0) {
        return field_count_is(frame, 2u) && valid_token(f0) && valid_token(frame->fields[1]) &&
               (strcmp(f0, "PREPARE") == 0 || strcmp(f0, "START") == 0 ||
                strcmp(f0, "PAUSE") == 0 || strcmp(f0, "RESUME") == 0 ||
                strcmp(f0, "ABORT") == 0 || strcmp(f0, "FINISH") == 0);
    }
    if (strcmp(frame->type, "CMD") == 0) {
        if (strcmp(f0, "START") == 0 || strcmp(f0, "STOP") == 0 ||
            strcmp(f0, "ESTOP") == 0 || strcmp(f0, "ARM") == 0) {
            return field_count_is(frame, 1u);
        }
        if (strcmp(f0, "DRIVE") == 0) {
            return field_count_is(frame, 4u) && parse_integer(frame->fields[1], -999, 999) &&
                   parse_integer(frame->fields[2], -999, 999) &&
                   parse_integer(frame->fields[3], 1, 400);
        }
        if (strcmp(f0, "MODE") == 0) {
            return field_count_is(frame, 2u) &&
                   (strcmp(frame->fields[1], "AUTO") == 0 || strcmp(frame->fields[1], "MANUAL") == 0);
        }
        if (strcmp(f0, "TARGET") == 0) {
            return field_count_is(frame, 3u) &&
                   parse_integer(frame->fields[1], INT32_MIN, INT32_MAX) &&
                   parse_integer(frame->fields[2], INT32_MIN, INT32_MAX);
        }
        return false;
    }
    if (strcmp(frame->type, "ACK") == 0) {
        return field_count_is(frame, 2u) && valid_hex(frame->fields[0], 8u) &&
               strcmp(frame->fields[1], "OK") == 0;
    }
    if (strcmp(frame->type, "NACK") == 0) {
        return field_count_is(frame, 2u) && valid_hex(frame->fields[0], 8u) &&
               valid_token(frame->fields[1]);
    }
    if (strcmp(frame->type, "HEARTBEAT") == 0) {
        return field_count_is(frame, 3u) &&
               (strcmp(f0, "PING") == 0 || strcmp(f0, "PONG") == 0) &&
               parse_integer(frame->fields[1], 0, INT32_MAX) && valid_token(frame->fields[2]);
    }
    if (strcmp(frame->type, "STATE") == 0) {
        return field_count_is(frame, 4u) && valid_token(frame->fields[0]) &&
               (strcmp(frame->fields[1], "AUTO") == 0 || strcmp(frame->fields[1], "MANUAL") == 0) &&
               valid_token(frame->fields[2]) &&
               valid_hex(frame->fields[3], 2u) &&
               (strcmp(frame->fields[3], "00") == 0 || strcmp(frame->fields[3], "01") == 0);
    }
    if (strcmp(frame->type, "POSE") == 0) {
        return field_count_is(frame, 4u) &&
               parse_integer(frame->fields[0], INT32_MIN, INT32_MAX) &&
               parse_integer(frame->fields[1], INT32_MIN, INT32_MAX) &&
               parse_integer(frame->fields[2], -180000, 180000) &&
               parse_integer(frame->fields[3], 0, INT32_MAX);
    }
    if (strcmp(frame->type, "CLAIM") == 0) {
        return field_count_is(frame, 3u) &&
               parse_integer(frame->fields[0], INT32_MIN, INT32_MAX) &&
               parse_integer(frame->fields[1], INT32_MIN, INT32_MAX) &&
               parse_integer(frame->fields[2], 0, INT32_MAX);
    }
    if (strcmp(frame->type, "VERDICT") == 0) {
        return field_count_is(frame, 3u) && valid_hex(frame->fields[0], 8u) &&
               (strcmp(frame->fields[1], "ACCEPT") == 0 || strcmp(frame->fields[1], "REJECT") == 0) &&
               valid_token(frame->fields[2]);
    }
    if (strcmp(frame->type, "OPPONENT") == 0) {
        return field_count_is(frame, 5u) && valid_token(frame->fields[0]) &&
               parse_integer(frame->fields[1], INT32_MIN, INT32_MAX) &&
               parse_integer(frame->fields[2], INT32_MIN, INT32_MAX) &&
               valid_hex(frame->fields[3], 8u) && parse_integer(frame->fields[4], 0, INT32_MAX);
    }
    if (strcmp(frame->type, "SENSOR") == 0) {
        return field_count_is(frame, 1u) && valid_hex(frame->fields[0], 2u);
    }
    if (strcmp(frame->type, "SPEED") == 0) {
        return field_count_is(frame, 2u) && parse_integer(frame->fields[0], -32768, 32767) &&
               parse_integer(frame->fields[1], -32768, 32767);
    }
    if (strcmp(frame->type, "EVENT") == 0) {
        return field_count_is(frame, 2u) && valid_token(frame->fields[0]) &&
               parse_integer(frame->fields[1], INT32_MIN, INT32_MAX);
    }
    if (strcmp(frame->type, "SCORE") == 0) {
        return field_count_is(frame, 3u) && parse_integer(frame->fields[0], 0, INT32_MAX) &&
               parse_integer(frame->fields[1], 0, INT32_MAX) && valid_token(frame->fields[2]);
    }
    return false;
}

uint16_t erth_v2_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFFu;
    size_t i;
    if (data == NULL && length != 0u) {
        return 0u;
    }
    for (i = 0u; i < length; ++i) {
        uint8_t bit;
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0u; bit < 8u; ++bit) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

bool erth_v2_seq_is_newer(uint32_t candidate, uint32_t previous) {
    uint32_t delta = candidate - previous;
    return delta != 0u && delta < 0x80000000u;
}

uint32_t erth_v2_seq_next(uint32_t *sequence) {
    uint32_t current;
    if (sequence == NULL) return 0u;
    current = *sequence;
    *sequence = current + 1u;
    return current;
}

erth_v2_result_t erth_v2_frame_init(erth_v2_frame_t *frame,
                                    const char *sid,
                                    const char *src,
                                    const char *dst,
                                    uint32_t seq,
                                    const char *type) {
    if (frame == NULL || sid == NULL || src == NULL || dst == NULL || type == NULL) {
        return ERTH_V2_ERR_ARGUMENT;
    }
    memset(frame, 0, sizeof(*frame));
    if (!copy_text(frame->sid, sizeof(frame->sid), sid) ||
        !copy_text(frame->src, sizeof(frame->src), src) ||
        !copy_text(frame->dst, sizeof(frame->dst), dst) ||
        !copy_text(frame->type, sizeof(frame->type), type)) {
        return ERTH_V2_ERR_FORMAT;
    }
    frame->seq = seq;
    return ERTH_V2_OK;
}

erth_v2_result_t erth_v2_add_field(erth_v2_frame_t *frame, const char *field) {
    if (frame == NULL || field == NULL) {
        return ERTH_V2_ERR_ARGUMENT;
    }
    if (frame->field_count >= ERTH_V2_MAX_FIELDS) {
        return ERTH_V2_ERR_OVERFLOW;
    }
    if (!copy_text(frame->fields[frame->field_count], ERTH_V2_FIELD_BYTES, field)) {
        return ERTH_V2_ERR_FORMAT;
    }
    ++frame->field_count;
    return ERTH_V2_OK;
}

static erth_v2_result_t validate_frame(const erth_v2_frame_t *frame) {
    size_t i;
    if (frame == NULL || !valid_hex(frame->sid, 16u) ||
        !valid_id(frame->src) || !valid_id(frame->dst) || strcmp(frame->src, frame->dst) == 0 ||
        !valid_token(frame->type) || frame->field_count > ERTH_V2_MAX_FIELDS) {
        return ERTH_V2_ERR_FORMAT;
    }
    for (i = 0u; i < frame->field_count; ++i) {
        if (!valid_token(frame->fields[i])) {
            return ERTH_V2_ERR_FORMAT;
        }
    }
    return validate_schema(frame) ? ERTH_V2_OK : ERTH_V2_ERR_SCHEMA;
}

erth_v2_result_t erth_v2_encode(const erth_v2_frame_t *frame,
                                uint8_t *output,
                                size_t output_capacity,
                                size_t *output_length) {
    char prefix[ERTH_V2_MAX_FRAME_BYTES];
    size_t used = 0u;
    size_t i;
    int written;
    uint16_t crc;
    static const char hex_digits[] = "0123456789ABCDEF";
    erth_v2_result_t result;

    if (frame == NULL || output == NULL || output_length == NULL) {
        return ERTH_V2_ERR_ARGUMENT;
    }
    result = validate_frame(frame);
    if (result != ERTH_V2_OK) {
        return result;
    }
    written = snprintf(prefix, sizeof(prefix), "2,%s,%s,%s,%08X,%s",
                       frame->sid, frame->src, frame->dst,
                       (unsigned int)frame->seq, frame->type);
    if (written < 0 || (size_t)written >= sizeof(prefix)) {
        return ERTH_V2_ERR_OVERFLOW;
    }
    used = (size_t)written;
    for (i = 0u; i < frame->field_count; ++i) {
        size_t field_length = strlen(frame->fields[i]);
        if (used + 1u + field_length >= sizeof(prefix)) {
            return ERTH_V2_ERR_OVERFLOW;
        }
        prefix[used++] = ',';
        memcpy(prefix + used, frame->fields[i], field_length);
        used += field_length;
        prefix[used] = '\0';
    }
    crc = erth_v2_crc16((const uint8_t *)prefix, used);
    if (output_capacity < used + 7u) {
        return ERTH_V2_ERR_OVERFLOW;
    }
    output[0] = '@';
    memcpy(output + 1u, prefix, used);
    output[1u + used] = '*';
    output[2u + used] = (uint8_t)hex_digits[(crc >> 12) & 0x0Fu];
    output[3u + used] = (uint8_t)hex_digits[(crc >> 8) & 0x0Fu];
    output[4u + used] = (uint8_t)hex_digits[(crc >> 4) & 0x0Fu];
    output[5u + used] = (uint8_t)hex_digits[crc & 0x0Fu];
    output[6u + used] = '\n';
    *output_length = used + 7u;
    if (*output_length > ERTH_V2_MAX_FRAME_BYTES) {
        return ERTH_V2_ERR_OVERFLOW;
    }
    return ERTH_V2_OK;
}

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

erth_v2_result_t erth_v2_decode(const uint8_t *input,
                                size_t input_length,
                                erth_v2_frame_t *frame) {
    char payload[ERTH_V2_MAX_FRAME_BYTES];
    char *tokens[ERTH_V2_MAX_TOKENS];
    size_t payload_length;
    size_t star = 0u;
    size_t token_count = 0u;
    size_t i;
    uint16_t expected_crc;
    uint16_t actual_crc;

    if (input == NULL || frame == NULL) {
        return ERTH_V2_ERR_ARGUMENT;
    }
    if (input_length == 0u || input_length > ERTH_V2_MAX_FRAME_BYTES) {
        return ERTH_V2_ERR_OVERFLOW;
    }
    if (input[input_length - 1u] != '\n') {
        return ERTH_V2_ERR_FORMAT;
    }
    if (input_length >= 2u && input[input_length - 2u] == '\r') {
        --input_length;
    }
    if (input_length < 10u || input[0] != '@') {
        return ERTH_V2_ERR_FORMAT;
    }
    for (i = 1u; i + 1u < input_length; ++i) {
        if (input[i] == '*') {
            if (star != 0u) return ERTH_V2_ERR_FORMAT;
            star = i;
        }
        if (input[i] > 0x7Fu || input[i] == '\r' || input[i] == '\n') {
            return ERTH_V2_ERR_FORMAT;
        }
    }
    if (star == 0u || input_length - star != 6u) {
        return ERTH_V2_ERR_FORMAT;
    }
    expected_crc = 0u;
    for (i = star + 1u; i < star + 5u; ++i) {
        int digit = hex_value((char)input[i]);
        if (digit < 0) return ERTH_V2_ERR_FORMAT;
        expected_crc = (uint16_t)((expected_crc << 4) | (uint16_t)digit);
    }
    actual_crc = erth_v2_crc16(input + 1u, star - 1u);
    if (actual_crc != expected_crc) {
        return ERTH_V2_ERR_CRC;
    }

    payload_length = star - 1u;
    memcpy(payload, input + 1u, payload_length);
    payload[payload_length] = '\0';
    tokens[token_count++] = payload;
    for (i = 0u; i < payload_length; ++i) {
        if (payload[i] == ',') {
            payload[i] = '\0';
            if (i + 1u >= payload_length || token_count >= ERTH_V2_MAX_TOKENS) {
                return ERTH_V2_ERR_FORMAT;
            }
            tokens[token_count++] = payload + i + 1u;
        }
    }
    if (token_count < 6u || strcmp(tokens[0], "2") != 0 ||
        strlen(tokens[1]) >= ERTH_V2_SID_BYTES || strlen(tokens[2]) >= ERTH_V2_ID_BYTES ||
        strlen(tokens[3]) >= ERTH_V2_ID_BYTES || strlen(tokens[5]) >= ERTH_V2_TYPE_BYTES ||
        !valid_hex(tokens[4], 8u)) {
        return ERTH_V2_ERR_FORMAT;
    }
    memset(frame, 0, sizeof(*frame));
    if (!copy_text(frame->sid, sizeof(frame->sid), tokens[1]) ||
        !copy_text(frame->src, sizeof(frame->src), tokens[2]) ||
        !copy_text(frame->dst, sizeof(frame->dst), tokens[3]) ||
        !copy_text(frame->type, sizeof(frame->type), tokens[5])) {
        return ERTH_V2_ERR_FORMAT;
    }
    frame->seq = 0u;
    for (i = 0u; i < 8u; ++i) {
        frame->seq = (frame->seq << 4) | (uint32_t)hex_value(tokens[4][i]);
    }
    frame->field_count = (uint8_t)(token_count - 6u);
    for (i = 0u; i < frame->field_count; ++i) {
        if (strlen(tokens[6u + i]) >= ERTH_V2_FIELD_BYTES ||
            !copy_text(frame->fields[i], ERTH_V2_FIELD_BYTES, tokens[6u + i])) {
            return ERTH_V2_ERR_FORMAT;
        }
    }
    return validate_frame(frame);
}

void erth_v2_stream_init(erth_v2_stream_t *stream) {
    if (stream != NULL) {
        memset(stream, 0, sizeof(*stream));
    }
}

void erth_v2_stream_feed(erth_v2_stream_t *stream,
                         const uint8_t *data,
                         size_t length,
                         erth_v2_frame_callback_t callback,
                         void *context) {
    size_t i;
    if (stream == NULL || (data == NULL && length != 0u)) {
        return;
    }
    for (i = 0u; i < length; ++i) {
        uint8_t byte = data[i];
        if (byte == '\n') {
            if (stream->length > 0u) {
                erth_v2_frame_t frame;
                if (stream->length < ERTH_V2_MAX_FRAME_BYTES) {
                    stream->bytes[stream->length++] = byte;
                    if (erth_v2_decode(stream->bytes, stream->length, &frame) == ERTH_V2_OK) {
                        if (callback != NULL) callback(&frame, context);
                    } else {
                        ++stream->error_count;
                    }
                } else {
                    ++stream->error_count;
                }
                stream->length = 0u;
            }
            continue;
        }
        if (byte == '@') {
            if (stream->length > 0u) ++stream->error_count;
            stream->length = 1u;
            stream->bytes[0] = byte;
            continue;
        }
        if (stream->length == 0u) continue;
        if (stream->length >= ERTH_V2_MAX_FRAME_BYTES - 1u) {
            ++stream->error_count;
            stream->length = 0u;
            continue;
        }
        stream->bytes[stream->length++] = byte;
    }
}

static erth_v2_result_t add_formatted(erth_v2_frame_t *frame, const char *format, long long value) {
    char text[ERTH_V2_FIELD_BYTES];
    int length = snprintf(text, sizeof(text), format, value);
    if (length < 0 || (size_t)length >= sizeof(text)) return ERTH_V2_ERR_OVERFLOW;
    return erth_v2_add_field(frame, text);
}

erth_v2_result_t erth_v2_make_hello(erth_v2_frame_t *frame,
                                    const char *vehicle_id,
                                    const char *firmware,
                                    const char *boot_id,
                                    const char *capabilities,
                                    uint8_t sensor_count) {
    erth_v2_result_t result;
    result = erth_v2_frame_init(frame, "0000000000000000", vehicle_id, "HUB", 0u, "HELLO");
    if (result != ERTH_V2_OK) return result;
    if ((result = erth_v2_add_field(frame, firmware)) != ERTH_V2_OK ||
        (result = erth_v2_add_field(frame, boot_id)) != ERTH_V2_OK ||
        (result = erth_v2_add_field(frame, capabilities)) != ERTH_V2_OK) return result;
    return add_formatted(frame, "%lld", sensor_count);
}

erth_v2_result_t erth_v2_make_ready(erth_v2_frame_t *frame,
                                    const char *sid,
                                    const char *vehicle_id,
                                    uint32_t seq,
                                    const char *status) {
    erth_v2_result_t result = erth_v2_frame_init(frame, sid, vehicle_id, "HUB", seq, "READY");
    if (result != ERTH_V2_OK) return result;
    return erth_v2_add_field(frame, status);
}

erth_v2_result_t erth_v2_make_ack(erth_v2_frame_t *frame,
                                  const char *sid,
                                  const char *src,
                                  const char *dst,
                                  uint32_t seq,
                                  uint32_t referenced_seq) {
    char reference[9];
    erth_v2_result_t result = erth_v2_frame_init(frame, sid, src, dst, seq, "ACK");
    if (result != ERTH_V2_OK) return result;
    (void)snprintf(reference, sizeof(reference), "%08X", (unsigned int)referenced_seq);
    if ((result = erth_v2_add_field(frame, reference)) != ERTH_V2_OK) return result;
    return erth_v2_add_field(frame, "OK");
}

erth_v2_result_t erth_v2_make_nack(erth_v2_frame_t *frame,
                                   const char *sid,
                                   const char *src,
                                   const char *dst,
                                   uint32_t seq,
                                   uint32_t referenced_seq,
                                   const char *error_code) {
    char reference[9];
    erth_v2_result_t result = erth_v2_frame_init(frame, sid, src, dst, seq, "NACK");
    if (result != ERTH_V2_OK) return result;
    (void)snprintf(reference, sizeof(reference), "%08X", (unsigned int)referenced_seq);
    if ((result = erth_v2_add_field(frame, reference)) != ERTH_V2_OK) return result;
    return erth_v2_add_field(frame, error_code);
}

erth_v2_result_t erth_v2_make_pose(erth_v2_frame_t *frame,
                                   const char *sid,
                                   const char *vehicle_id,
                                   uint32_t seq,
                                   int32_t x10,
                                   int32_t y10,
                                   int32_t yaw_mdeg,
                                   uint32_t uptime_ms) {
    erth_v2_result_t result = erth_v2_frame_init(frame, sid, vehicle_id, "HUB", seq, "POSE");
    if (result != ERTH_V2_OK) return result;
    if ((result = add_formatted(frame, "%lld", x10)) != ERTH_V2_OK ||
        (result = add_formatted(frame, "%lld", y10)) != ERTH_V2_OK ||
        (result = add_formatted(frame, "%lld", yaw_mdeg)) != ERTH_V2_OK) return result;
    return add_formatted(frame, "%lld", uptime_ms);
}

erth_v2_result_t erth_v2_make_claim(erth_v2_frame_t *frame,
                                    const char *sid,
                                    const char *vehicle_id,
                                    uint32_t seq,
                                    int32_t x10,
                                    int32_t y10,
                                    uint32_t uptime_ms) {
    erth_v2_result_t result = erth_v2_frame_init(frame, sid, vehicle_id, "HUB", seq, "CLAIM");
    if (result != ERTH_V2_OK) return result;
    if ((result = add_formatted(frame, "%lld", x10)) != ERTH_V2_OK ||
        (result = add_formatted(frame, "%lld", y10)) != ERTH_V2_OK) return result;
    return add_formatted(frame, "%lld", uptime_ms);
}
