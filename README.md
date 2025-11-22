## RTSerial - Real-Time Serial Communication Library

**RTSerial** 是一个专为 Linux 环境设计的轻量级、高性能 C++ 串口通信库。它采用 **Header-only**（仅头文件）设计，通过 C++ 模板技术支持自定义数据包结构，并内置了独立的接收线程和环形缓冲区，特别适合机器人控制、嵌入式通信等对实时性要求较高的场景。

## ✨ 主要特性

  * **轻量级集成**：单头文件 (`RTSerial.hpp`)，无须编译静态/动态库，直接引用即可使用。
  * **模板化设计**：支持用户自定义 `Packet` 结构体，自动处理结构体大小。
  * **多线程接收**：内置独立的后台接收线程，与主业务逻辑解耦。
  * **低延迟策略**：接收线程采用“最新帧优先”策略（自动 Flush 旧数据），确保获取的是当前最新的传感器/硬件状态。
  * **环形缓冲区**：线程安全的环形队列，防止读写冲突。
  * **校验回调**：支持注入自定义的校验函数（CRC、SumCheck 等），自动丢弃非法包。
  * **灵活配置**：支持波特率、数据位、停止位、奇偶校验位的详细配置。

## 🛠️ 依赖环境

  * **OS**: Linux (依赖 `<termios.h>`, `<sys/ioctl.h>` 等系统调用)
  * **Compiler**: C++11 及以上 (支持 `std::thread`, `std::atomic`, `std::unique_ptr`)
  * **Build System**: CMake, Makefile 或直接 g++ 均可。

## 🚀 快速开始

### 1\. 定义通信协议

首先定义你的数据包结构。建议使用 `__attribute__((packed))` 确保字节对齐（或者手动处理填充）。

**注意**：`RTSerial` 默认校验数据包的第一个字节作为帧头。

```cpp
#include <cstdint>

// 示例数据包：总长 16 字节

struct __attribute__((packed)) RobotData {
    uint8_t header;      // 帧头 (例如 0xA5)
    float pitch;         // 数据1
    float yaw;           // 数据2
    uint32_t timestamp;  // 时间戳
    uint8_t checksum;    // 校验位
};
#pragma pack(pop)
```

### 2\. 简单接收示例

```cpp
#include "RTSerial.hpp"
#include <iostream>

int main() {
    // 1. 实例化串口对象
    // 缓冲区大小设为 5，帧头定义为 0xA5
    RTSerial<RobotData> serial(5, 0xA5);

    // 2. 打开串口
    // 设备: /dev/ttyUSB0, 波特率: 115200
    if (serial.openDevice("/dev/ttyUSB0", 115200) != 1) {
        std::cerr << "Failed to open serial port!" << std::endl;
        return -1;
    }

    // 3. (可选) 设置校验函数
    // 如果返回 false，该包会被丢弃
    serial.setCheckfuc([](const RobotData& pkt) {
        // 简单的示例：检查 header 是否匹配 (虽然底层已经检查过 header 字节)
        return pkt.header == 0xA5; 
    });

    // 4. 开启后台接收线程
    serial.startReceive();

    std::cout << "Start receiving..." << std::endl;

    RobotData currentData;
    RTSerial<RobotData>::timePoint recvTime;

    while (true) {
        // 5. 从缓冲区读取最新的包
        if (serial.readPacket(currentData, recvTime)) {
            std::cout << "Pitch: " << currentData.pitch 
                      << " Yaw: " << currentData.yaw << std::endl;
        }
        
        // 模拟业务处理耗时
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    serial.closeDevice();
    return 0;
}
```

### 3\. 发送示例

```cpp
RobotData sendPkg;
sendPkg.header = 0xA5;
sendPkg.pitch = 1.5f;
// ... 填充数据

// 发送整个结构体
serial.writeBytes(&sendPkg, sizeof(RobotData));
```

## 📚 API 说明

### 构造函数

```cpp
RTSerial(size_t buffSize = 3, uint8_t head = 0xA5);
```

  * `buffSize`: 内部环形缓冲区的容量（包的数量）。
  * `head`: 用于识别一帧数据开始的标识字节。

### 核心方法

  * `int openDevice(...)`: 打开并配置串口（波特率、停止位等）。
  * `void startReceive(interval_us, timeOut_ms)`: 启动后台线程。
      * `interval_us`: 最小读取间隔，防止 CPU 占用过高。
      * `timeOut_ms`: 读取单个包体剩余数据的超时时间。
  * `void setCheckfuc(...)`: 设置校验回调 lambda。
  * `bool readPacket(Packet &data, ...)`: 从缓冲区取出一个完整的包。
  * `bool writeBytes(...)`: 发送原始字节数据。
  * `void closeDevice()`: 停止线程并关闭文件描述符。

## ⚠️ 注意事项

1.  **丢包策略 (Drop Policy)**：
    `startReceive` 线程的设计逻辑是“实时性优先”。在每次读取前，它会调用 `flushReceiver()` 清空系统内核缓冲区。这意味着如果主程序处理过慢，积压在系统底层的旧数据会被直接丢弃，只读取刚刚到达的数据。这对于机器人姿态控制是合理的，但如果用于传输文件或关键指令，请需谨慎。

2.  **内存对齐**：
    由于直接将串口字节流 `memcpy` 到结构体，请务必保证发送端（如 STM32/嵌入式）和接收端（PC）的结构体**内存布局一致**（字节对齐方式、大小端序）。

3.  **权限问题**：
    Linux 下操作 `/dev/tty*` 通常需要权限，请将用户加入 `dialout` 组或使用 `sudo` 运行：

    ```bash
    sudo usermod -aG dialout $USER
    ```
    或者临时权限：
    ```bash
    sudo chmod 777 /dev/tty*
    ```

## 📝 License

该项目基于 MIT License 开源（参考源文件头部声明）。
Author: Philippe Lucidarme (Original), Modified for RT Applications.