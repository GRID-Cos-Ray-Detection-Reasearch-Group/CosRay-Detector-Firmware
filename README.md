# CosRay FreeRTOS

## 项目简介

CosRay FreeRTOS 是一个基于 ESP32S3 微控制器的宇宙射线检测项目，使用 FreeRTOS 实时操作系统。该项目旨在检测宇宙射线（主要是 μ 介子），并通过 GPS、PPS 和 Muon 中断等机制收集和处理数据。项目支持数据存储、蓝牙通信和远程传输功能。

该项目基于开源硬件和软件，允许用户构建自己的宇宙射线检测器，并连接到网络以共享数据。

## 软件要求

- ESP-IDF (Espressif IoT Development Framework)
- FreeRTOS (通过 ESP-IDF 组件提供)
- CMake (用于构建)

## 构建和运行

### 环境设置

1. 安装 ESP-IDF：
   - 推荐使用 VsCode + [ESP-IDF 扩展](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)
   - 手动安装，详细步骤请参考 [ESP-IDF 官方文档](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)。

2. 设置代码格式化环境（可选，但推荐）：

   本项目使用 pre-commit 和 clang-format 来自动格式化 C 代码。
   - 运行一键设置脚本（根据你的操作系统选择）：
     - Windows PowerShell：`.\setup.ps1`
     - Windows CMD：`setup.bat`
     - Linux/Mac：`./setup.sh`

     这些脚本会自动安装 uv 和 pre-commit，并设置 Git 钩子。

   - 手动设置：
     - 安装 uv：`curl -LsSf https://astral.sh/uv/install.sh | sh`
     - 安装 pre-commit：`uv pip install pre-commit`
     - 安装钩子：`pre-commit install`

3. 克隆项目仓库：

   ```bash
   git clone https://github.com/ADCirx/CosRay_FreeRTOS.git
   cd CosRay_FreeRTOS
   ```

4. 配置 ESP-IDF 环境

### 构建项目

1. 配置 sdkconfig：

   ```bash
   idf.py menuconfig
   ```

2. 构建项目：

   ```bash
   idf.py build
   ```

3. 烧录到 ESP32S3：

   ```bash
   idf.py flash
   ```

4. 监控输出：

   ```bash
   idf.py monitor
   ```

### 注意事项

- git 仓库会忽略 `build`、`Archives` 和 `Binaries` 目录下的所有文件，因此每次从远程仓库同步后需要重新构建。
- ESP-IDF 自带组件的配置全部通过工程下 `sdkconfig` 完成，如果对 `sdkconfig` 有改动，也应当新建分支进行测试。

## 开发规范

### 协作开发

1. **分支管理**：每次实现新功能前，从远程仓库拉取最新版本代码到本地仓库，再新建一个分支。实现功能后，推送分支并提交 Pull Request。

2. **Commit 规范**：
   - 使用约定式提交信息
   - 参考 [Conventional Commits](https://www.conventionalcommits.org/zh-hans/v1.0.0/) 规范。
   - 示例：`feat: 实现 GPS 同步`，`fix: 修复 pre-commit 格式化错误`。

3. **Issue 提交**：提交 bug 或 feature issue 请按照仓库内 issue template 格式。

### 代码规范

1. **代码格式化**：本项目使用多种格式化工具进行代码格式化。提交前会自动运行格式化检查，确保代码风格一致。
   - C/C++ 文件：使用 clang-format，规则定义在 `.clang-format` 文件中。
   - Python 文件：使用 Black。
   - Markdown、JSON、YAML 等文件：使用 Prettier。

2. **命名规范**：自定义变量名称使用英文或英文缩写，尽可能简短而清晰。使用大驼峰命名法（PascalCase），每个逻辑断点的单词首字母大写，无需下划线。示例：存储科学数据的缓存命名为 `SCIBuf`。

## 许可证

本项目采用 MPL 2.0 许可证。修改不可闭源，但新增代码协议不必延续 MPL 协议。

## 参考资料

- [ESP-IDF 官方文档](https://docs.espressif.com/projects/esp-idf/)
- [FreeRTOS 官方文档](https://www.freertos.org/)
- [宇宙射线检测相关研究](https://www.researchgate.net/publication/394806196_Ultra_high_energy_cosmic_ray_detection)
- [开源宇宙射线检测器 Cosmic Pi](https://cosmicpi.org/)

## 贡献

欢迎提交 Issue 和 Pull Request。请遵循上述开发规范。
