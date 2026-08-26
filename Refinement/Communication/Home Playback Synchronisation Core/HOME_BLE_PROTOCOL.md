# Home-side BLE Protocol

All multibyte integers are little-endian. CRC is CRC-16/CCITT-FALSE over every byte before the `crc16` field.

## Feedback Command, 20 bytes

| Offset | Field | Type |
|---:|---|---|
| 0 | type/version, `0xC1` | UInt8 |
| 1 | command, 1 START, 2 UPDATE, 3 STOP | UInt8 |
| 2 | flags, bit 0 = resume | UInt8 |
| 3 | stop reason | UInt8 |
| 4 | event ID | UInt32 |
| 8 | command sequence | UInt16 |
| 10 | CES, value / 255 | UInt8 |
| 11 | peak CES, value / 255 | UInt8 |
| 12 | onset Spike, value / 255 | UInt8 |
| 13 | maximum duration, deciseconds | UInt16 |
| 15 | remaining duration, deciseconds | UInt16 |
| 17 | reserved | UInt8 |
| 18 | CRC16 | UInt16 |

## Command ACK, 12 bytes

| Offset | Field | Type |
|---:|---|---|
| 0 | type/version, `0xB1` | UInt8 |
| 1 | command | UInt8 |
| 2 | status | UInt8 |
| 3 | feedback state | UInt8 |
| 4 | event ID | UInt32 |
| 8 | command sequence | UInt16 |
| 10 | CRC16 | UInt16 |

## Home Telemetry, 20 bytes

| Offset | Field | Type |
|---:|---|---|
| 0 | type/version, `0xD1` | UInt8 |
| 1 | device ID | UInt8 |
| 2 | status flags | UInt8 |
| 3 | feedback state | UInt8 |
| 4 | telemetry sequence | UInt16 |
| 6 | ESP32 timestamp ms | UInt32 |
| 10 | BPM | UInt8 |
| 11 | heart response, value / 255 | UInt8 |
| 12 | motion score, value / 255 | UInt8 |
| 13 | pitch / 2 degrees | Int8 |
| 14 | roll / 2 degrees | Int8 |
| 15 | PTC 1 temperature × 2 °C | Int8 |
| 16 | PTC 2 temperature × 2 °C | Int8 |
| 17 | fault flags | UInt8 |
| 18 | CRC16 | UInt16 |

## Maintenance Control, 6 bytes

| Offset | Field | Type |
|---:|---|---|
| 0 | command, 1 recalibrate, 2 clear thermal faults, 3 emergency stop | UInt8 |
| 1 | protocol version, 1 | UInt8 |
| 2 | request ID | UInt16 |
| 4 | CRC16 | UInt16 |
