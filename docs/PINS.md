# ESP32-S3 N16R8 引脚分配

工程：`VESC-TOOLS-ESP32`  
芯片：ESP32-S3（16MB Flash + 8MB OPI PSRAM）  
定义文件：`src/pins.h`（油门/PPM 另在 `src/main.cpp`）

---

## 功能引脚一览

| GPIO | 功能 | 方向 | 说明 |
|------|------|------|------|
| 0 | 板载 BOOT | — | **固件不使用**（留给下载/复位） |
| 1 | 油门 ADC | 模拟输入 | 电位器中点 → GPIO1 |
| 2 | PPM / PWM 输出 | 输出 | 单路舵机 PWM 或 4 通道 CPPM |
| 4 | VESC UART RX | 输入 | ← 电调 TX，115200 |
| 5 | VESC UART TX | 输出 | → 电调 RX |
| 7 | TFT 背光 BL | PWM | ST7789 |
| 8 | TFT RST | 输出 | |
| 9 | TFT DC | 输出 | |
| 10 | TFT CS | 输出 | |
| 11 | TFT MOSI (SDA) | SPI | |
| 12 | TFT SCK (SCL) | SPI | |
| 14 | 按键 DOWN | 输入上拉 | 共地，按下为低 |
| 15 | 按键 LEFT | 输入上拉 | |
| 16 | 按键 RIGHT | 输入上拉 | |
| 17 | 按键 OK | 输入上拉 | |
| 18 | GPS UART TX | 输出 | → GPS RX |
| 19 | USB D− | 板载 | **勿接外设** |
| 20 | USB D+ | 板载 | **勿接外设** |
| 21 | GPS UART RX | 输入 | ← GPS TX，默认 230400（可自动扫描） |
| 26–37 | Flash / PSRAM | Octal SPI | **禁止使用** |
| 38 | SDMMC CMD | SD | 板载 microSD（1-bit） |
| 39 | SDMMC CLK | SD | |
| 40 | SDMMC D0 | SD | 勿改 4-bit（会碰到 43） |
| 43 | CH343 USB-UART | 板载 | **勿接外设** |
| 44 | CH343 USB-UART | 板载 | **勿接外设** |
| 46 | 按键 UP | 输入上拉 | **strapping 脚**；上电/复位时勿按住 |

---

## 按模块接线

### TFT 2.4" ST7789（SPI）

| 模块脚 | ESP32 |
|--------|-------|
| GND | GND |
| VCC | 3.3V |
| SCL | GPIO12 |
| SDA | GPIO11 |
| RST | GPIO8 |
| DC | GPIO9 |
| CS | GPIO10 |
| BL | GPIO7（或常亮接 3.3V） |

### 五向按键（共地，active-LOW）

| 键 | GPIO |
|----|------|
| UP | **46**（勿用 13：紧挨 TFT SCK 易误触发） |
| DOWN | 14 |
| LEFT | 15 |
| RIGHT | 16 |
| OK | 17 |
| COM | GND |

### GPS（M10 等 UART）

| 信号 | GPIO |
|------|------|
| ESP RX ← GPS TX | 21 |
| ESP TX → GPS RX | 18 |
| VCC | 建议 5V（按模块要求） |
| GND | 共地 |

### VESC UART

| 信号 | GPIO |
|------|------|
| ESP TX → VESC RX | 5 |
| ESP RX ← VESC TX | 4 |
| GND | 共地 |
| 波特率 | 115200 |

### 油门与输出

| 信号 | GPIO |
|------|------|
| 电位器中点（ADC） | 1 |
| PPM/PWM 出 | 2 |
| 电位器两端 | 3.3V / GND（按硬件） |

### 板载 microSD（1-bit SDMMC）

| 信号 | GPIO |
|------|------|
| CLK | 39 |
| CMD | 38 |
| D0 | 40 |

---

## 相对空闲（可扩外设）

常见可用：**3, 6, 13, 41, 42, 45\*, 47, 48**

\* GPIO45 也是 strapping，慎用。  
GPIO13 空闲可用，但靠近 TFT SCK，做按键需注意干扰。

---

## 菜单操作（五向键）

- 主界面：短按/长按 **OK** → 设置  
- 设置项：
  1. TRIP STATS — 里程/最高速  
  2. THROTTLE CAL — 油门三点校准  
  3. **OUT PWM/CPPM** — OK 切换输出模式  
  4. **CPPM CH n** — OK 切换油门通道（1–4；PWM 下会先切到 CPPM）  
  5. SD TEST — 测卡  
  6. BACK — 返回  
- **上/下**（右键同上）选择，**OK** 进入/切换，**左** 返回  
- 板载 BOOT **不参与**模式切换（避免误触/异常）
