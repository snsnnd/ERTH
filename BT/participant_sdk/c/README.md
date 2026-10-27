# ERTH Battle V2 — Embedded C SDK

This is a portable, allocation-free C99 codec for participant firmware. It has no STM32 HAL dependency: the application supplies UART/BLE reads and writes, and uses the stream parser to receive frames.

## Files

- `include/erth_battle_v2.h` — public API and fixed-size frame structures.
- `src/erth_battle_v2.c` — CRC, encode/decode, stream framing, and common HELLO/READY/ACK/POSE/CLAIM builders.
- `examples/basic_client.c` — transport/HAL integration outline.
- `tests/test_erth_battle_v2.c` — host-side golden-frame and stream test.

## Build and test on a host

```sh
gcc -std=c99 -Wall -Wextra -Werror -pedantic \
  -I include src/erth_battle_v2.c tests/test_erth_battle_v2.c \
  -o erth_v2_test
./erth_v2_test
```

For STM32, add `src/erth_battle_v2.c` to the firmware build and include `include/erth_battle_v2.h`. The transport should pass the received bytes to `erth_v2_stream_feed()` and transmit the exact byte count returned by `erth_v2_encode()`.

## Minimal use

```c
erth_v2_frame_t frame;
uint8_t tx[ERTH_V2_MAX_FRAME_BYTES];
size_t tx_length;
char boot_id[9];

board_get_boot_id(boot_id); /* board hook: new persistent boot counter per reset */
boot_id[8] = '\0';
erth_v2_make_hello(&frame, "A", "2.0.0", boot_id, "DRIVE|POSE|CLAIM", 8);
if (erth_v2_encode(&frame, tx, sizeof(tx), &tx_length) == ERTH_V2_OK) {
    uart_write(tx, tx_length); /* application-provided transport function */
}
```

The `boot_id` must change on every reset, not merely identify the MCU. The example delegates it to `board_get_boot_id()`; implement that hook with a persistent boot counter or another reset-varying source and return exactly eight uppercase hexadecimal characters.

Use `erth_v2_stream_feed()` for arbitrary UART DMA/interrupt chunks. Its callback receives a pointer to a temporary frame valid only during the callback; copy the structure if it must be retained. The SDK validates framing and schema, but the application remains responsible for session checks, match-state gates, ACK behavior, motor leases, and fail-safe stopping.

## Safety requirements

The SDK is not a motor controller and does not make unsafe firmware safe by itself. Stop the motors on boot, reset, transport loss, invalid/stale session, ESTOP, and DRIVE lease expiry. Do not run V1 and V2 parsers on the same connection at the same time.

Protocol details and participant rules are in `../../docs/PLAYER_PROTOCOL_V2.md`.
