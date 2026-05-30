# 底盘二进制协议设计

## 帧格式

| 字段 | 大小 | 说明 |
| --- | --- | --- |
| magic | 2 byte | 固定 `0xAA55` |
| version | 1 byte | 当前为 `1` |
| type | 1 byte | 消息类型 |
| seq | 2 byte | 帧序号 |
| payload_len | 2 byte | payload 长度 |
| payload | N byte | 消息负载 |
| crc16 | 2 byte | CRC16-CCITT，覆盖 header 和 payload |

多字节数值采用 little-endian 编码。

## 消息类型

- `0x01`：`CmdVelPayload`，包含 `float vx_mps`、`float wz_radps`
- `0x02`：`OdomFeedbackPayload`，包含位姿、速度和左右轮速度
- `0x03`：心跳帧

## 校验

解码时校验 magic、version、payload 长度、CRC 和消息类型。payload 最大长度为 128 byte。
