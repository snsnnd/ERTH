# ERTH Battle Protocol V2 — 选手接入说明

**版本：2.0 候选｜状态：BT 平台协议层与 C SDK 初版已实现；选手车辆固件接入和实车联调待完成**

本文定义选手车辆与 BT 对战平台之间的 V2 通信格式和安全要求。SPP 虚拟串口与 BLE UART 使用相同协议。本文不定义比赛计分、胜负条件或角色策略；这些规则需由赛事组织方另行公布。

> V2 与当前 V1 文本命令不兼容。V1 的 `DRV:`、`STOP`、`ESTOP`、`START`、`POS:`、`RPOS:`、`ORP:` 等不能直接用于 V2 正式比赛。V2 未完成双方实现和验收前，请继续使用 V1 调试模式。

## 1. 接入要求

车辆通过串口或 BLE UART 与平台连接。车辆端必须：

1. 上电、复位、断线重连时保持电机停止；完成 V2 握手前不得执行运动命令。
2. 只接受平台发来的控制和比赛状态；车辆通过 `STATE`、`POSE`、`CLAIM` 等消息上报自身状态。
3. 每条 V2 帧都进行 CRC 校验、字段/范围校验和会话校验；无效帧不得驱动车辆。
4. 实现驱动租约：`DRIVE` 到期即停车。无线断开或收不到刷新帧时，不得保持上一次 PWM。
5. 收到 `ESTOP` 后锁存停车；只有平台在允许状态下显式发送 `ARM` 才能解除锁存。`ARM` 不会启动电机。

平台是唯一的对局主机。正式比赛的开始、暂停和恢复作用于整场；练习模式可由平台单独控制一辆车。CRC 只能检测传输损坏，不能认证发送者或防止作弊。

## 2. 传输规则

- UART：115200 baud、8 data bits、no parity、1 stop bit（8N1）。
- 一帧以 `LF`（字节 `0x0A`）结束；接收端可兼容 `CRLF`。
- BLE 的写入/通知可能分片或粘连。必须先按字节累积，再按 `LF` 组帧；一次 BLE 操作不等于一帧。
- 最大帧长为 **128 字节，包含 `LF`**。超长帧整帧丢弃，并重新寻找下一个 `@`。
- 协议内容为 ASCII；字段内不能出现逗号、`*`、`@`、CR 或 LF。不得把调试日志混入协议链路。

## 3. 帧格式

```text
@2,<sid>,<src>,<dst>,<seq>,<type>,<field1>,...*<crc16>\n
```

| 字段 | 格式与含义 |
|---|---|
| `2` | 协议主版本，固定为 ASCII `2`。未知版本必须拒绝。 |
| `sid` | 64-bit 会话号，16 位大写十六进制。握手前固定为 `0000000000000000`；收到 `WELCOME` 后必须使用平台分配的新会话号。 |
| `src` / `dst` | `HUB`、`A`、`B`。车辆身份由平台按物理连接配置绑定，车辆不得自行更换身份。 |
| `seq` | 32-bit 发送序号，8 位大写十六进制；每个发送端在当前会话内独立递增，允许自然回绕。新会话重新从 `00000000` 开始。 |
| `type` | 大写消息类型；载荷字段数量和顺序固定，见第 5 节。 |
| `crc16` | 4 位大写十六进制 CRC-16/CCITT-FALSE。覆盖从版本字符 `2` 到 `*` 前最后一个载荷字节，包含逗号；不包含 `@`、`*`、CRC 字符和行结束符。 |

CRC 参数：poly `0x1021`、init `0xFFFF`、输入/输出不反射、xorout `0x0000`。

### 示例

以下 CRC 为实际计算值；示例会话号仅供说明：

```text
@2,0000000000000000,A,HUB,00000000,HELLO,2.0.0,ABCDEF01,DRIVE|POSE|CLAIM,8*4CD9\n
@2,7B42E19A03D50211,HUB,A,00000011,CMD,STOP*A807\n
@2,7B42E19A03D50211,A,HUB,00000001,ACK,00000011,OK*2DAE\n
```

解析器遇到 CRC 错误、字段错误、超长、错误版本或非 ASCII 字节时必须丢弃该帧。旧 `sid` 的帧不得执行；应丢弃并重新握手，不能用旧会话响应。

## 4. 会话与序号

1. 车辆停车后发送 `HELLO`，其 `sid` 为全零，车辆本次启动的 HELLO 序号从 0 开始；`boot_id` 必须在每次启动时变化，用于区分复位/重启。
2. 平台按物理端口将该连接绑定为 A 或 B；验证成功后发送 `WELCOME`，其中包含新 `sid` 和本局配置。收到 WELCOME 前车辆不得运动。
3. 车辆校验配置后，使用新 `sid` 发送 `READY`；平台 ACK 后握手完成。车辆从此只接受当前 `sid` 的帧。
4. 每个发送端的序号独立递增。相同 `sid`、相同 `seq` 的重复请求不得重复执行；需确认的请求必须重发原 ACK。序号按 32-bit 模数顺序判断；平台仅为可靠性处理允许 ACK/NACK 在 16 个序号窗口内迟到，旧遥测/状态帧不得回放。
5. 需要确认的请求超时 300 ms 后，发送端以**相同序号和相同帧内容**重发，最多重发 3 次；仍未确认时平台将链路视为故障。除 `DRIVE` 租约刷新流外，所有状态改变命令和 MATCH 命令都需要 ACK/NACK；ACK/NACK 本身不需要确认。

## 5. 消息定义

以下字段均为 ASCII；整数使用十进制文本，除非明确标为十六进制。固定字段不能省略。消息方向中的“车辆”指 A 或 B。

| 类型 | 方向 | 载荷字段（按顺序）与说明 |
|---|---|---|
| `HELLO` | 车辆 → HUB | `fw,boot_id,caps,sensor_count`。`boot_id` 为每次启动变化的 8 位大写 HEX（建议使用持久化启动计数）；`caps` 为以 `|` 分隔的能力名（没有能力填 `-`），能力名如 `DRIVE`、`POSE`、`CLAIM`、`SENSOR8`、`SPEED`。 |
| `WELCOME` | HUB → 车辆 | `role,map_version,hb_ms,lease_ms,mode`。`mode` 为 `OFFICIAL` 或 `BENCH`。`map_version` 为 `NONE`（未加载地图）或地图源文件 UTF-8 内容 SHA-256 的前 8 位大写 HEX。它分配新会话并下发配置。 |
| `READY` | 车辆 → HUB | `status`，为 `OK` 或 `NOT_READY_<reason>`。`OK` 表示车辆已核对并应用 WELCOME 配置。 |
| `MATCH` | HUB → 车辆 | `action,reason`。`action` 为 `PREPARE`、`START`、`PAUSE`、`RESUME`、`ABORT` 或 `FINISH`。比赛生命周期命令；START/PAUSE/RESUME/FINISH/ABORT 作用于整场。无原因时 `reason` 填 `NONE`。 |
| `CMD` | HUB → 车辆 | 一种车辆动作：`START`（仅 BENCH 单车启动）；`DRIVE,left_pwm,right_pwm,lease_ms`（周期刷新，不逐帧 ACK）；`STOP`；`ESTOP`；`ARM`；`MODE,AUTO|MANUAL`；`TARGET,x10,y10`。除 DRIVE 外均需 ACK/NACK；动作受当前对局状态和车辆能力限制。 |
| `ACK` | 双向 | `ref_seq,result`。确认原请求；`ref_seq` 为被确认帧的 8 位序号，`result` 成功时为 `OK`。 |
| `NACK` | 双向 | `ref_seq,error_code`。拒绝原请求；错误码是固定大写 token，如 `BAD_STATE`、`BAD_RANGE`、`ESTOP_LATCHED`、`UNSUPPORTED`。 |
| `HEARTBEAT` | 双向 | `PING|PONG,uptime_ms,state`。`uptime_ms` 为发送端单调毫秒计数；`state` 为当前状态 token。 |
| `STATE` | 车辆 → HUB | `car_state,control_mode,role,flags`。`car_state` 至少支持 `PARKED`、`STOPPED`、`MOVING`、`ESTOP`；`control_mode` 为 `AUTO` 或 `MANUAL`；`flags` 是 2 位 HEX，最低位 bit0 表示急停锁存，其他位当前保留并置 0。车辆必须在运动/模式/急停状态改变，以及处理完 `STOP`、`ESTOP`、`ARM` 或比赛状态命令后上报 STATE。 |
| `POSE` | 车辆 → HUB | `x10,y10,yaw_mdeg,uptime_ms`。位置单位为地图单位的十分之一；航向单位毫度，范围 `-180000..180000`。 |
| `CLAIM` | 车辆 → HUB | `x10,y10,uptime_ms`。车辆申报位置；平台会校验，未经裁决不能视为可信或转发给对手。 |
| `VERDICT` | HUB → 车辆 | `claim_seq,ACCEPT|REJECT,reason`。裁决对应的 CLAIM 序号；拒绝/延迟原因使用固定 token。 |
| `OPPONENT` | HUB → 车辆 | `opponent_id,x10,y10,claim_seq,age_ms`。仅包含平台已接受且未过期的对手申报位置。 |
| `SENSOR` | 车辆 → HUB | `line_mask`，可选 8-bit 循迹快照，2 位 HEX。bit 对应的传感器顺序由赛事提供的硬件配置说明定义。 |
| `SPEED` | 车辆 → HUB | `left_mmps,right_mmps`，左右轮速度，单位 mm/s。 |
| `EVENT` | 车辆 → HUB | `event_code,value`。仅报告观测；不能由车辆直接修改比分。 |
| `SCORE` | HUB → 车辆 | `team_a,team_b,reason`。只有赛事组织方公布计分规则后才启用；当前版本保留，不代表已定义比赛得分。 |

`CMD` 的不同动作载荷长度不同，解析时必须先检查子命令，再按该子命令的固定字段数解析。接收方不认识的消息/子命令应返回 `UNSUPPORTED`（握手前无法建立响应会话时直接丢弃）。

## 6. 控车与安全行为

- `CMD,DRIVE,left_pwm,right_pwm,lease_ms` 的 PWM 范围为 `-999..999`；`lease_ms` 不得超过 WELCOME 下发的上限，当前建议上限 400 ms、平台刷新周期约 100 ms。
- `CMD,MODE` 与 `CMD,TARGET` 只配置运行模式/目标，不会单独使能电机；平台可在 LOBBY/PREPARING/PAUSED 配置它们。`DRIVE` 仅在 BENCH 单车已启动或官方比赛 RUNNING 且控制模式为 MANUAL 时接受。
- 驱动租约到期后车辆清零电机并保持停车；不得自动切换到 AUTO。
- `CMD,STOP` 是可恢复软停车；`CMD,ESTOP` 是任何状态都可触发的锁存急停。停止/急停优先于普通运动命令。
- `CMD,ARM` 只解除急停锁存，不启动电机；仅在车辆已停车且状态允许时接受。官方比赛随后还需合法的整场 `MATCH,START` / `MATCH,RESUME`；BENCH 才能由平台单独发送 `CMD,START`。
- 平台进入 FAULT 后，选手固件须在执行停车后上报最新 `STATE`；平台确认停车命令并收到 FAULT 之后的停车状态，才允许人工复位回 LOBBY。
- 官方比赛中不得单独启动一辆车。对运行中单车 STOP/ESTOP，平台默认暂停整场并停车另一辆车。
- 建议 HEARTBEAT 周期 250 ms；1 秒无有效帧视为离线。比赛中一车掉线默认暂停整场；重连必须重新握手和 READY，不得恢复旧驱动命令。
- 位置过期、缺少地图或无法裁决时平台必须拒绝/暂缓转发，不能以未经定义的估算替代裁决。申报位置距离阈值和计分规则由赛事配置另行公布。

## 7. 当前未由本协议指定的赛事规则

本协议不自行规定角色能力、人工驾驶是否允许、申报位置最大图距离、计分/胜负条件、地图布局或传感器 bit 顺序。选手应以本场比赛的正式规则和硬件配置为准；平台在规则未配置前不得据此自动判分。

## 8. 选手库

`participant_sdk/c/` 已提供不依赖 STM32 HAL 的嵌入式 C99 库初版，负责帧编码/解码、CRC、分片组帧、序号和常用消息构造。选手固件通过 UART/BLE 收发回调接入，不需要自行重写帧格式。库只负责协议，不替代车辆电机安全策略、硬件急停或掉线停车逻辑。车辆固件接入和实车验收仍待完成。
