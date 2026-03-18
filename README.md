# wristband

## protocol

little-endian

| Field | Magic | Length                      | Version  | Message ID | Command | Data                   | Crc          |
| ----- | ----- | --------------------------- | -------- | ---------- | ------- | ---------------------- | ------------ |
| size  | 1BYTE | 2BYTE                       | 4BYTE    | 4BYTE      | 2BYTE   | (len-[magic-crc]) BYTE | 2BYTE        |
|       | 0xAA  | 包含magic-crc整个数据的长度 | 协议版本 | 消息ID     |         |                        | crc16-modbus |

### command

| Command  | Name                    | Data size（Byte） | Description             |
| :------- | :---------------------- | :---------------- | :---------------------- |
| `0x01` | BLOCK_DATA_SEMG         | 48                | 16-channel sEMG data    |
| `0x02` | BLOCK_DATA_IMU          | 36                | 9-axis IMU data         |
| `0x03` | BLOCK_DATA_CAMERA_UP    | -                 | Camera data (reserved)  |
| `0x04` | BLOCK_DATA_CAMERA_DOWN  | -                 | Camera data (reserved)  |
| `0x05` | BLOCK_DATA_VIBRATE_DATA | -                 | Vibration feedback data |
| `0x06` | BLOCK_DATA_BATTERY      | -                 | Battery status          |
