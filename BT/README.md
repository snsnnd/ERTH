# ERTH BT 对战平台

BT 当前保留 V1 调试模式，并新增 V2 双车对局协议实现。V2 的选手接入规范见 [`docs/PLAYER_PROTOCOL_V2.md`](docs/PLAYER_PROTOCOL_V2.md)；架构与机制说明见 [`docs/BATTLE_PROTOCOL_V2.md`](docs/BATTLE_PROTOCOL_V2.md)。

## 运行平台

安装 `requirements.txt` 中依赖后运行：

```sh
python final.py
```

每路串口连接可独立选择 V1 或 V2；BLE 连接也可选择 V1/V2，并绑定为 A 或 B。V1 是默认选项，不会自动升级旧固件。

### V2 BENCH 单车练习

1. 在“V2 对局模式”选择 `BENCH`。
2. 连接车辆时将协议选为 V2；车辆须先完成 `HELLO/WELCOME/READY`。
3. 选中 A、B 或已绑定角色的 BLE 目标，按“解除急停”（需要时）后按“启动车辆”。
4. 使用方向键进行手动控制；松开方向键发送零速驱动刷新，停车/急停按钮通过 V2 命令发送。

### V2 OFFICIAL 双车对局

1. 先加载比赛地图；地图版本是其 UTF-8 源文本 SHA-256 的前 8 位大写 HEX。
2. 选择 `OFFICIAL`，再用 V2 协议连接 A、B。若更改地图或对局模式，需要重新连接，使新配置进入 WELCOME。
3. 两车均 READY 且在线后依次执行“准备比赛”“开始比赛”。开始/暂停/恢复/结束由平台整场控制。
4. 比赛中对单车软停或触发急停，平台默认暂停整场；故障状态需车辆停车、链路恢复后人工复位。

V2 当前没有获批的计分/胜负规则，因此不自动判分。CRC 不是身份认证或防作弊机制。未完成选手固件接入和双车实车验收前，请勿用于正式比赛。

## 代码与选手库

- `battle_protocol/` — Python V2 帧编解码、会话管理和平台比赛控制层。
- `participant_sdk/c/` — 不依赖 STM32 HAL 的嵌入式 C99 选手库、示例与主机测试。
- `tests/` — Python 协议与平台状态机测试。

当前平台 UI 默认仍使用 V1。V2 必须由操作员明确选择，且车辆固件也必须实现本目录下的 V2 选手协议。
