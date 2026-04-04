# ESP32-S3 I2C 加速度传感器状态检测

本工程用于在 ESP32-S3 上检查 I2C 加速度传感器是否在线，并尝试读取常见传感器的 ID 寄存器。

## 硬件连接
- SCL: GPIO1
- SDA: GPIO2
- 供电: 按传感器模组要求接 3.3V 和 GND

建议确保 I2C 总线上有上拉电阻（常见为 4.7k 到 10k）。

## 编译与烧录（ESP-IDF）
1. 设置目标芯片为 esp32s3
2. 编译
3. 烧录并监视串口日志

> 注意：在 Windows 下，ESP-IDF 某些版本会在工程路径包含中文字符时出现编码错误（如 gbk decode 失败）。
> 如果遇到该问题，请把工程移动到纯英文路径（例如 `D:\esp\acce`）后再构建。

在 VS Code 的 ESP-IDF 插件中，可依次执行：
- Set Espressif Device Target
- Build your Project
- Flash your Project
- Monitor your Device

## 日志判定
- 扫描到设备地址，且可读取到寄存器: 传感器基本在线
- 未扫描到地址: 检查连线、供电、上拉电阻、地址脚配置
- 扫描到地址但读取失败: 检查器件寄存器定义、I2C 频率或电平兼容性
