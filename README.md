Cosmic Ray detection with ESP32S3， running freertos， ver.0.1

MPL2.0许可协议，修改不可闭源，但新增代码协议不必延续MPL协议

---

## 快速上手 / Build & Flash

### 环境要求

- **ESP-IDF v5.3.x**（推荐 5.3.1）
- 目标芯片：**ESP32-S3**
- Python 3.12+

### 编译步骤

```bash
# 1. 设置目标芯片
idf.py set-target esp32s3

# 2. 编译（首次会根据 sdkconfig.defaults 自动补全配置）
idf.py build

# 3. 烧录
idf.py -p /dev/ttyUSBx flash monitor
```

> **注意**：如果 `sdkconfig` 不存在，运行 `idf.py set-target esp32s3` 会用 `sdkconfig.defaults` 生成初始配置。
> 如果遇到 BT 未启用问题，删除 `sdkconfig` 后重新运行上述步骤。

### 已实现功能

| 功能                               | 状态 | 说明                                    |
| ---------------------------------- | ---- | --------------------------------------- |
| SiPM 偏压（LT3482 DCDC + SPI DAC） | TODO | 引脚预留，驱动待实现                    |
| μ子触发中断 + ADC 采样             | ✅   | GPIO10 触发，ADC 采集信号幅度           |
| GPS 时间同步（UBX NAV-PVT）        | ✅   | UART1 RX=GPIO17，PPS=GPIO11             |
| GPS PPS 精确时间轴                 | ✅   | PPS 中断 + CPU 时钟计数                 |
| Flash 存储（W25N01KV SPI NAND）    | ✅   | SPI2，MOSI=GPIO42，MISO=GPIO47          |
| 蓝牙 BLE 数据传输（NimBLE）        | ✅   | 设备名 "MuonDetector"，自定义 GATT 服务 |
| μ子事件数据包 + 时间线数据包       | ✅   | 每35个事件打包，每5秒时间线快照         |

### 硬件引脚分配

| 信号              | GPIO   | 说明              |
| ----------------- | ------ | ----------------- |
| ADC 信号输入      | GPIO4  | SiPM 信号幅度采样 |
| TMP112 温度报警   | GPIO5  | 双沿中断          |
| SiPM 阴极电压监测 | GPIO6  | ADC 采样          |
| SiPM 电流监测     | GPIO7  | ADC 采样          |
| μ子触发（比较器） | GPIO10 | 上升沿中断        |
| GPS PPS           | GPIO11 | 上升沿中断        |
| GPS UART RX       | GPIO17 | UART1             |
| GPS UART TX       | GPIO18 | UART1             |
| Flash CS          | GPIO35 | SPI2              |
| Flash CLK         | GPIO48 | SPI2              |
| Flash MOSI        | GPIO42 | SPI2              |
| Flash MISO        | GPIO47 | SPI2              |
| Flash WP          | GPIO36 | SPI2              |
| Flash HOLD        | GPIO38 | SPI2              |

### BLE 服务说明

- **设备名**：`MuonDetector`
- **服务 UUID**：`01234567-89ab-cdef-0102-030405060708`
- **控制特征**（WRITE）：发送 `CommandPkg_t`（8字节命令 + 2字节CRC）
  - `0x01` START：开始数据采集
  - `0x02` STOP：停止采集
  - `0x03/0x04` ACK/NACK：数据确认
- **数据特征**（NOTIFY）：推送 `MuonDataPkg_t` 或 `TimeLinePkg_t`（512字节）

---

1.1. git仓库会忽略build与Archives与Binaries目录下的所有文件，因此每次从远程仓库同步后需要重新build。

1.2. 协作开发方式：每次实现新功能前，从远程仓库拉取（pull）最新版本代码到本地仓库，再新建一个分支（branch）。本地仓库多次
commit后，编译测试实现了期望的功能，把本地仓库push到新建分支上，同时在push时提交一份说明。之后再提交一个pull request，
申请将新分支与main分支合并，经过其他成员测试审核后通过合并请求，正式合并。

1.3. commit is cheap，commit 仅提交代码到本地仓库，因此为了版本管理方便起见应该多commit。但每次commit应该简单描述
commit的改动和期望实现的目的，以便维护一个完整的change log。

1.4. 提交bug issue与featrue issue请按照仓库内issue template格式。

git使用细节参考（https://zhuanlan.zhihu.com/p/51199833 ）

2.1. FreeRTOS直接使用ESP-IDF组件，因此没有包含在工程目录下，需要正确配置ESP-IDF。

2.2. ESP-IDF自带组件的配置全部通过工程下sdkconfig完成，因此如果对sdkconfig有改动，也应当新建branch进行测试。

3.1. 自定义的变量名称应当使用英文或英文缩写，尽可能简短而清晰地说明这个变量的功能。具体命名规范为大驼峰命名法，
（参考 https://blog.csdn.net/weixin_43758823/article/details/84888470 ），每个逻辑断点的单词的首字母大写，不需要
下划线。示例：需要设置一个存储科学数据的缓存，命名为SCIBuf。

3.2. main.c内定义的函数仅包含任务函数、中断服务子程。自定义子函数应当在BSP文件夹下新开.c和.h文件。

3.3. 在bsp.h内include BSP文件夹内所有其它.h文件，这样只需要在.c文件内 include一次bsp.h，就能够使用BSP内所有自定义函数。

3.4. 写.h文件时注意使用#ifndef #define #endif语句，来避免.h文件重复include。

3.5. 所有全局变量都在main.c内定义，并在bsp.h内通过extern关键字声明，以此避免重复声明。如果全局变量使用了自定义结构体，
结构体也在bsp.h内声明。如果有.c文件需要用到全局变量，就include bsp.h。

示例：

在bsp.h内声明SCIBuf结构体
typedef struct SCIPkg {
uint8_t Head[3];
SCIDataType Event;
SCIDataShortType EventShort[43];
uint8_t EffCnt[4];
uint8_t LostCnt[4];
uint8_t Tail[3];
uint8_t CRC[2];
}SCIPkgType;

在bsp.h内通过extern关键字声明SCIBuf全局变量

extern SCIPkgType SCIBuf[2][8];

在main.c内定义SCIBuf全局变量

SCIPkgType SCIBuf[2][8] = {0};

3.6. 作者信息与代码功能在代码内通过注释体现，示例：

/_
High flux mode global var (edit by LLH 2024.11.21)
_/

1.2. 协作开发方式：每次实现新功能前，从远程仓库拉取（pull）最新版本代码到本地仓库，再新建一个分支（branch）。本地仓库多次
commit后，编译测试实现了期望的功能，把本地仓库push到新建分支上，同时在push时提交一份说明。之后再提交一个pull request，
申请将新分支与main分支合并，经过其他成员测试审核后通过合并请求，正式合并。

1.3. commit is cheap，commit 仅提交代码到本地仓库，因此为了版本管理方便起见应该多commit。但每次commit应该简单描述
commit的改动和期望实现的目的，以便维护一个完整的change log。

1.4. 提交bug issue与featrue issue请按照仓库内issue template格式。

git使用细节参考（https://zhuanlan.zhihu.com/p/51199833 ）

2.1. FreeRTOS直接使用ESP-IDF组件，因此没有包含在工程目录下，需要正确配置ESP-IDF。

2.2. ESP-IDF自带组件的配置全部通过工程下sdkconfig完成，因此如果对sdkconfig有改动，也应当新建branch进行测试。

3.1. 自定义的变量名称应当使用英文或英文缩写，尽可能简短而清晰地说明这个变量的功能。具体命名规范为大驼峰命名法，
（参考 https://blog.csdn.net/weixin_43758823/article/details/84888470 ），每个逻辑断点的单词的首字母大写，不需要
下划线。示例：需要设置一个存储科学数据的缓存，命名为SCIBuf。

3.2. main.c内定义的函数仅包含任务函数、中断服务子程。自定义子函数应当在BSP文件夹下新开.c和.h文件。

3.3. 在bsp.h内include BSP文件夹内所有其它.h文件，这样只需要在.c文件内 include一次bsp.h，就能够使用BSP内所有自定义函数。

3.4. 写.h文件时注意使用#ifndef #define #endif语句，来避免.h文件重复include。

3.5. 所有全局变量都在main.c内定义，并在bsp.h内通过extern关键字声明，以此避免重复声明。如果全局变量使用了自定义结构体，
结构体也在bsp.h内声明。如果有.c文件需要用到全局变量，就include bsp.h。

示例：

在bsp.h内声明SCIBuf结构体
typedef struct SCIPkg {
uint8_t Head[3];
SCIDataType Event;
SCIDataShortType EventShort[43];
uint8_t EffCnt[4];
uint8_t LostCnt[4];
uint8_t Tail[3];
uint8_t CRC[2];
}SCIPkgType;

在bsp.h内通过extern关键字声明SCIBuf全局变量

extern SCIPkgType SCIBuf[2][8];

在main.c内定义SCIBuf全局变量

SCIPkgType SCIBuf[2][8] = {0};

3.6. 作者信息与代码功能在代码内通过注释体现，示例：

/_
High flux mode global var (edit by LLH 2024.11.21)
_/
