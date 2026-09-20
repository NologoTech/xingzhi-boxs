# 无名科技星智 S3 4G（无屏）

无名科技星智 S3 4G 板卡，基于 ESP32-S3 + ES8311 + ML307，无显示屏。默认 4G，开机阶段可双击切换到 WiFi。

## 硬件要点

- 芯片：ESP32-S3
- 网络：ML307 Cat.1 4G（TX `GPIO12` / RX `GPIO11`，供电使能 `GPIO21`）；支持双击切换 WiFi
- 音频：ES8311（I2S + I2C；GPIO21 用于 4G 供电，PA 由 codec 内部控制）
- 按键：BOOT `GPIO8`（开机启动/配网阶段双击切换 4G↔WiFi；WiFi 模式下开机单击进入配网）
- 状态灯：WS2812 `GPIO48`（空闲灭、连接蓝、聆听红、播放绿）
- 电源：USB/电池 ADC 检测，电源键关机控制

## 编译

```bash
python3 scripts/build.py nologo/xingzhi-s3-4g --name xingzhi-s3-4g
```
