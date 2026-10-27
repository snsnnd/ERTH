#ifndef ERTH_BATTLE_V2_H
#define ERTH_BATTLE_V2_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ERTH_V2_MAX_FRAME_BYTES 128u
#define ERTH_V2_MAX_FIELDS 8u
#define ERTH_V2_FIELD_BYTES 48u
#define ERTH_V2_SID_BYTES 17u
#define ERTH_V2_ID_BYTES 4u
#define ERTH_V2_TYPE_BYTES 16u

typedef enum {
    ERTH_V2_OK = 0,
    ERTH_V2_ERR_ARGUMENT,
    ERTH_V2_ERR_OVERFLOW,
    ERTH_V2_ERR_FORMAT,
    ERTH_V2_ERR_CRC,
    ERTH_V2_ERR_SCHEMA
} erth_v2_result_t;

typedef struct {
    char sid[ERTH_V2_SID_BYTES];
    char src[ERTH_V2_ID_BYTES];
    char dst[ERTH_V2_ID_BYTES];
    uint32_t seq;
    char type[ERTH_V2_TYPE_BYTES];
    uint8_t field_count;
    char fields[ERTH_V2_MAX_FIELDS][ERTH_V2_FIELD_BYTES];
} erth_v2_frame_t;

typedef struct {
    uint8_t bytes[ERTH_V2_MAX_FRAME_BYTES];
    size_t length;
    uint32_t error_count;
} erth_v2_stream_t;

typedef void (*erth_v2_frame_callback_t)(const erth_v2_frame_t *frame, void *context);

uint16_t erth_v2_crc16(const uint8_t *data, size_t length);
bool erth_v2_seq_is_newer(uint32_t candidate, uint32_t previous);
uint32_t erth_v2_seq_next(uint32_t *sequence);

erth_v2_result_t erth_v2_frame_init(erth_v2_frame_t *frame,
                                    const char *sid,
                                    const char *src,
                                    const char *dst,
                                    uint32_t seq,
                                    const char *type);
erth_v2_result_t erth_v2_add_field(erth_v2_frame_t *frame, const char *field);
erth_v2_result_t erth_v2_encode(const erth_v2_frame_t *frame,
                                uint8_t *output,
                                size_t output_capacity,
                                size_t *output_length);
erth_v2_result_t erth_v2_decode(const uint8_t *input,
                                size_t input_length,
                                erth_v2_frame_t *frame);

void erth_v2_stream_init(erth_v2_stream_t *stream);
void erth_v2_stream_feed(erth_v2_stream_t *stream,
                         const uint8_t *data,
                         size_t length,
                         erth_v2_frame_callback_t callback,
                         void *context);

erth_v2_result_t erth_v2_make_hello(erth_v2_frame_t *frame,
                                    const char *vehicle_id,
                                    const char *firmware,
                                    const char *boot_id,
                                    const char *capabilities,
                                    uint8_t sensor_count);
erth_v2_result_t erth_v2_make_ready(erth_v2_frame_t *frame,
                                    const char *sid,
                                    const char *vehicle_id,
                                    uint32_t seq,
                                    const char *status);
erth_v2_result_t erth_v2_make_ack(erth_v2_frame_t *frame,
                                  const char *sid,
                                  const char *src,
                                  const char *dst,
                                  uint32_t seq,
                                  uint32_t referenced_seq);
erth_v2_result_t erth_v2_make_nack(erth_v2_frame_t *frame,
                                   const char *sid,
                                   const char *src,
                                   const char *dst,
                                   uint32_t seq,
                                   uint32_t referenced_seq,
                                   const char *error_code);
erth_v2_result_t erth_v2_make_pose(erth_v2_frame_t *frame,
                                   const char *sid,
                                   const char *vehicle_id,
                                   uint32_t seq,
                                   int32_t x10,
                                   int32_t y10,
                                   int32_t yaw_mdeg,
                                   uint32_t uptime_ms);
erth_v2_result_t erth_v2_make_claim(erth_v2_frame_t *frame,
                                    const char *sid,
                                    const char *vehicle_id,
                                    uint32_t seq,
                                    int32_t x10,
                                    int32_t y10,
                                    uint32_t uptime_ms);

#ifdef __cplusplus
}
#endif

#endif
