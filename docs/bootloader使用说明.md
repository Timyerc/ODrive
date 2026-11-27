# bootloader 使用说明：CAN OTA 升级流程与原理

OTA 支持参数和应用两类镜像。远程系统负责升级控制，主控板只转发 CAN 帧；驱动器自行停机、准备暂存区并报告就绪。每个 OTA CAN 帧均等待驱动器回复后再继续。参数下载成功后由当前应用直接写入 NVM A 区，不重启；应用下载成功后写入暂存区元数据、回复整包 CRC 并复位进入 bootloader，由 bootloader 安装应用。

## 首次安装与日常升级

1. 在 `Firmware` 目录运行 `tup --quiet --no-environ-check`，同时生成两个镜像。
2. **首次部署用 SWD 写入两个镜像**：`build\bootloader\ODriveBootloader.bin` 写到 `0x08000000`，`build\ODriveFirmware.bin` 写到 `0x08010000`。bootloader 本身不接收 CAN 下载；仅烧录 bootloader 的空板不能通过本协议安装第一个应用。
3. 新协议不兼容旧版：迁移时通过 SWD 更新配套的 bootloader 和应用，不要只更新脚本。确认设备可安全停机、总线上只有一个目标、适配器与目标波特率一致。Windows PCAN-USB 需安装 PEAK 驱动和 PCAN-Basic；Python 端安装 `python -m pip install python-can`。
4. 从仓库根目录运行 `python tools\odrive\download_firmware.py`。文件名必须分别包含 `Firmware` 和 `Config`（不区分大小写），扩展名为 `.bin`。应用最大为应用 Flash 分区的 320 KiB，参数最大为 NVM 扇区的 128 KiB；不再设置应用文件的最小大小。应用仅检查复位向量入口位于应用分区，不检查栈指针。
5. 可分别下载，或点击“自动下载：参数 → 应用”。自动流程先擦除 NVM A/B、把参数写入 A 区并重建 NVM 索引，不重启；接着下载应用。设备返回完整镜像 CRC 并通过校验后，应用立即回复并复位进入 bootloader 安装应用。整个过程保持供电。
6. 如需验证分片重传，可在传输期间反复点击“模拟数据干扰”。每次注入都会对一个数据帧翻转数据位；该帧仍会收到逐帧应答，分片结束时 CRC 校验失败后工具重发未损坏的整片。该测试不会修改本地镜像文件。

命令行 `--type` 可选 `app`（默认）或 `parameters`。可用 `--auto --parameters-image <Config.bin> --application-image <Firmware.bin>` 自动按参数、应用顺序下载。单独下载时 `--size`、`--crc32`、`--chunks` 可省略。

```powershell
python tools\odrive\download_firmware.py Firmware\build\ODriveFirmware.bin --channel PCAN_USBBUS1 2>&1 | Tee-Object ota.log
```

```powershell
python tools\odrive\download_firmware.py --auto --parameters-image C:\update\Config.bin --application-image C:\update\ODriveFirmware.bin --channel PCAN_USBBUS1
```

读取当前驱动器版本号：

```powershell
python tools\odrive\download_firmware.py --read-version --channel PCAN_USBBUS1
```

## Flash 分区

| 扇区 | 起始地址 | 大小 | 用途 |
|---|---|---:|---|
| 0–2 | `0x08000000` | 48 KiB | bootloader；复位入口 |
| 3 | `0x0800C000` | 16 KiB | `BOOTLOADER_DATA_START`：OTA 参数与升级标志 |
| 4–6 | `0x08010000` | 320 KiB | `APPLICATION_START`：正在运行的应用 |
| 7–9 | `0x08060000` | 384 KiB | `APPLICATION_BUFFER_START`：暂存新镜像，不在此执行 |
| 10、11 | `0x080C0000`、`0x080E0000` | 各 128 KiB | 原 NVM 双缓冲；参数更新会擦除两扇区并写入 A 区 |

固件上限取**应用区的 320 KiB**，不是暂存区的 384 KiB。下载过程中旧应用仍在；开始安装时才擦除应用区。这不是双应用槽切换，覆盖后没有旧版本可自动回滚。

## CAN 时序：主控板如何发送一次升级

远程上位机使用经典 CAN **扩展帧 ID `0xABC`** 发送，驱动器使用扩展帧 ID `0xDEF` 回复，每帧最多 8 字节。大小、片数和 CRC32 均为小端。主控板仅转发，不自行安排升级步骤。收到升级请求后，驱动器自行让电机停机并擦除暂存区；CAN 中断保持工作，以便继续应答和接收。

| 顺序 | 上位机 → 驱动器：载荷（按线上字节顺序） | 驱动器应答 / 上位机动作 |
|---|---|---|
| 1a. 应用请求 | `10` + 固件大小(u24) | 请求共 4 字节。整包 CRC 不随请求发送；即使目标 CRC 相同也会重新下载。 |
| 1b. 参数请求 | `11` + 固件大小(u24) | 请求共 4 字节；参数最大 128 KiB。 |
| 2. 驱动器就绪 | 驱动器回复 `13` + 状态(u8) + 总片数(u16) + `00 00 00 00` | 状态 `1` 表示停机、准备和暂存区擦除完成；上位机核对片数后开始发送。状态 `0` 表示仍在准备（重复请求时返回），`2` 表示拒绝请求。 |
| 3. 发送数据帧 | `01` + 片内帧号(u8) + 6 字节数据 | 每片 1024 字节，帧号 `0…170`。最后一帧超出片长的数据补 `FF`。每发一帧都必须等待对应应答后再继续。 |
| 4. 片内帧应答 | 驱动器回复 `02` + 片内帧号(u8) + `00 00 00 00 00` | 共 7 字节，只表示该帧已收到，不带状态。上位机收到对应帧号的应答后才能继续下一帧；等待超时则重发同一帧。 |
| 5. 查询分片 CRC | 收完帧号 `170` 的应答后，上位机发送 `03` + 本片 CRC32(u32) | 驱动器回复 `04` + 实际分片 CRC32(u32)。匹配时才写入暂存区并推进片号；不匹配时驱动器丢弃当前片，上位机重发整片。 |
| 6. 查询整包 CRC | 所有分片确认后，上位机发送 `05` + 本地整包 CRC32(u32) | 驱动器计算暂存区真实固件长度的 CRC，并回复 `06` + 实际整包 CRC32(u32)。上位机比对一致后才显示下载完成。 |
| 7. 查询固件版本 | 上位机发送 `07` | 驱动器回复 `08` + 主版本(u8) + 次版本(u8) + 修订版本(u8) + 未发布标志(u8)。各字节来自构建生成的 `version.c`；未发布标志 `1` 表示开发版本，`0` 表示正式版本。 |

收到 `06` 且整包 CRC 匹配后，参数镜像由驱动器写入 NVM A 区，不重启；应用镜像由驱动器提交 OTA 元数据并复位进入 bootloader。片内逐帧应答只确认接收，不代表分片或整包校验成功。

### CRC 校验范围

算法与 Python `zlib.crc32` 一致（CRC-32/ISO-HDLC）。**填充是否参与校验**取决于校验对象：

- **整包**：只算 `.bin` 的真实字节，不含填充。上位机在所有分片完成后通过 `05` 发送本地 CRC，驱动器计算暂存区 CRC 并以 `06` 回复；应用搬运后也会重新验证。
- **分片**：分片数为 `(真实大小 + 1023) // 1024`。末片用 `FF` 补到 1024 字节再算 CRC；片内最后一个 CAN 帧额外的 2 个 `FF` 不参与。例如 1025 字节的文件有 2 片，第二片 CRC 覆盖 1 个文件字节和 1023 个 `FF`。
- **OTA 元数据记录**：仅应用升级时使用，覆盖记录的前 24 字节，不含记录 CRC 自身和可变状态字。

## 应用镜像的 bootloader 安装

1. 收到 `05` 整包 CRC 查询后，应用计算真实长度的暂存区 CRC 并以 `06` 回复。CRC 与上位机提供值一致且复位入口位于应用区时，写入 32 字节 OTA 记录，将 `PENDING` 最后写入，然后在回复后立即复位。
2. bootloader 检查记录完整性、镜像物理边界、暂存整包 CRC 和应用复位入口地址。随后擦除应用扇区 4–6，复制暂存镜像并重新校验 CRC 与复位入口。
3. 安装成功后将 `PENDING` 变更为 `INSTALLED` 并复位。下次启动时 bootloader 设置 `SCB->VTOR` 为 `0x08010000`，由应用向量表设置 MSP 并跳转到复位入口。

## 中断升级与恢复边界

参数镜像保留 NVM 分配表，不新增头尾。请先备份 A/B 两个扇区并使用兼容的参数镜像。参数传输校验成功后，当前应用立即擦除两区、写入 A 区并调用 `NVM_init` 更新运行时索引；此操作不重启，也不经过 bootloader。当前运行中的配置对象不会自动重载，镜像配置在下次启动时生效。参数写入不是原子操作，过程中断电可能需要 SWD 恢复。CRC 仅验证数据一致，不代表配置参数适合该电机。

- **应用下载中断、尚未提交 `PENDING`**：应用区未擦除；重新启动设备并从头下载，不支持跨复位断点续传。
- **安装前检查失败**：bootloader 尝试写入 `FAILED`（`0xFFFFFFF8`），不因该镜像擦除应用区；后续跳转仍要求应用向量有效。
- **已开始擦写应用区**：如果擦写或搬运后校验失败且状态仍为 `PENDING`，bootloader 停留，不跳入可能残缺的应用。下次复位会再次检查暂存区并尝试安装。若持续失败或没有可启动的应用，使用 SWD 恢复，不要等待 bootloader 应答 CAN。
- CRC 检查传输和存储错误，**不验证发布者身份，也不阻止降级**。仅使用与目标硬件匹配、来源可信的固件。

实现对照：[分区、元数据与协议常量](../communication/bootloader.hpp) · [应用 CAN 接收状态机](../communication/can_download.cpp) · [bootloader 搬运与跳转](./main.cpp) · [主控板参考发送程序](../../tools/odrive/download_firmware.py)。
