# 项目长期记忆 — Anhua (Y75_Model, STM32F103CB)

## 构建 / 工具链（本机路径）
```bash
cd D:/ST/workspace/vscode/Anhua
export PATH="/d/ST/workspace/compiler/arm-gnu-toolchain-15.2.rel1-mingw-w64-x86_64-arm-none-eabi/bin:$PATH"
"C:/Users/y3010/AppData/Local/stm32cube/bundles/ninja/1.13.2+st.1/bin/ninja.exe" -C build/Debug
```
- cmake: `/d/ST/workspace/compiler/cmake-4.3.2-windows-x86_64/bin/cmake`（presets: Debug/Release）
- 已有 `build/Debug` 直接 ninja 增量编译即可；产物 `build/Debug/Y75_Model.hex|.bin`
- 烧录工具：`/d/ST/workspace/compiler/xpack-openocd-0.12.0-5`；另有 `Y75_Model Debug.launch`

## 硬件约定
- RS485 挂在 **USART1**（PA9/PA10），不是 USART2；调试打印 `uart_debug()` 与 485 数据同一路。
- 485 方向脚：**PA8 = RS485_DE**（1=发送/驱动总线，0=空闲），**PA11 = RS485_RE**
  （1=接收使能，0=发送时关闭以免自发自收）。空闲态 = DE 低 + RE 高。
- PA12 = 数据流指示灯（500 ms 翻转）。
- 数据结构：`AA 55 [CMD] [PAYLOAD] 0D 0A`；采样率 500 Hz / 1 kHz 可选，波特率默认 460800
  存 EEPROM(0x78)，输出单位存 0x7C。

## 编码约定
- CubeMX 生成文件里的改动都放在 `USER CODE BEGIN/END` 区域内。
- 所有 485 总线发送必须走 `rs485_transmit()`，否则 PA8/PA11 不会切换。
