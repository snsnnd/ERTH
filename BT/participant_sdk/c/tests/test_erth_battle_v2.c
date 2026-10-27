#include "erth_battle_v2.h"

#include <assert.h>
#include <string.h>

static void on_frame(const erth_v2_frame_t *frame, void *context) {
    unsigned int *count = (unsigned int *)context;
    assert(frame != NULL);
    assert(strcmp(frame->type, "HELLO") == 0);
    ++(*count);
}

int main(void) {
    static const uint8_t expected[] =
        "@2,0000000000000000,A,HUB,00000000,HELLO,2.0.0,ABCDEF01,DRIVE|POSE|CLAIM,8*4CD9\n";
    erth_v2_frame_t frame;
    erth_v2_frame_t decoded;
    erth_v2_stream_t stream;
    uint8_t output[ERTH_V2_MAX_FRAME_BYTES];
    size_t output_length = 0u;
    unsigned int callback_count = 0u;

    assert(erth_v2_crc16((const uint8_t *)"123456789", 9u) == 0x29B1u);
    assert(erth_v2_make_hello(&frame, "A", "2.0.0", "ABCDEF01", "DRIVE|POSE|CLAIM", 8u) == ERTH_V2_OK);
    assert(erth_v2_encode(&frame, output, sizeof(output), &output_length) == ERTH_V2_OK);
    assert(output_length == sizeof(expected) - 1u);
    assert(memcmp(output, expected, output_length) == 0);
    assert(erth_v2_decode(output, output_length, &decoded) == ERTH_V2_OK);
    assert(strcmp(decoded.type, "HELLO") == 0);
    assert(strcmp(decoded.fields[2], "DRIVE|POSE|CLAIM") == 0);

    erth_v2_stream_init(&stream);
    erth_v2_stream_feed(&stream, output, 17u, on_frame, &callback_count);
    assert(callback_count == 0u);
    erth_v2_stream_feed(&stream, output + 17u, output_length - 17u, on_frame, &callback_count);
    assert(callback_count == 1u);
    assert(erth_v2_seq_is_newer(0u, 0xFFFFFFFFu));
    {
        uint32_t sequence = 0xFFFFFFFFu;
        assert(erth_v2_seq_next(&sequence) == 0xFFFFFFFFu);
        assert(sequence == 0u);
    }
    return 0;
}
