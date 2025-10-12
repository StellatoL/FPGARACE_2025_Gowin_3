#include <iostream>
#include <windows.h>
#include <vector>
#include <atomic>
#include <mutex>
#include <tchar.h>
#include <algorithm>

// 常量定义
constexpr TCHAR SHARED_MEM_NAME[] = _T("CPP_PYTHON_SHARE");
constexpr TCHAR DATA_WRITTEN_EVENT_NAME[] = _T("CPP_PYTHON_EVENT");
constexpr int MAX_FLOATS = 64;
constexpr int BUFFER_SIZE = 2048;  // 增大缓冲区防止溢出
constexpr int BLOCK_SIZE = 64;     // 每个数据块64字节（64个8位数据点）

// 共享数据结构
#pragma pack(push, 1)
struct SharedData {
    std::atomic<int> num_floats;
    float data[MAX_FLOATS];
};
#pragma pack(pop)

// 高效环形缓冲区
class CircularBuffer {
public:
    CircularBuffer(size_t capacity)
        : buffer(capacity), capacity(capacity), head(0), tail(0) {}

    // 写入数据（线程安全）
    bool write(const unsigned char* data, size_t len) {
        std::lock_guard<std::mutex> lock(mtx);
        if (free_space() < len) return false;

        const size_t first_chunk = std::min(len, capacity - head);
        const size_t second_chunk = len - first_chunk;

        if (first_chunk > 0) {
            memcpy(&buffer[head], data, first_chunk);
            head = (head + first_chunk) % capacity;
        }

        if (second_chunk > 0) {
            memcpy(&buffer[head], data + first_chunk, second_chunk);
            head = (head + second_chunk) % capacity;
        }

        return true;
    }

    // 读取指定长度的数据块
    bool read_bytes(size_t len, std::vector<unsigned char>& out) {
        std::lock_guard<std::mutex> lock(mtx);
        if (data_size() < len) return false;

        out.resize(len);

        const size_t first_chunk = std::min(len, capacity - tail);
        const size_t second_chunk = len - first_chunk;

        if (first_chunk > 0) {
            memcpy(out.data(), &buffer[tail], first_chunk);
            tail = (tail + first_chunk) % capacity;
        }

        if (second_chunk > 0) {
            memcpy(out.data() + first_chunk, &buffer[tail], second_chunk);
            tail = (tail + second_chunk) % capacity;
        }

        return true;
    }

private:
    inline size_t data_size() const {
        return (head >= tail) ? (head - tail) : (capacity - tail + head);
    }

    inline size_t free_space() const {
        return capacity - data_size() - 1; // 保留一个字节防止满状态歧义
    }

    std::vector<unsigned char> buffer;
    size_t capacity;
    size_t head;
    size_t tail;
    std::mutex mtx;
};

int main() {
    // 创建共享内存
    HANDLE hMapFile = CreateFileMapping(
        INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
        0, sizeof(SharedData), SHARED_MEM_NAME);

    if (!hMapFile) {
        std::cerr << "CreateFileMapping failed: " << GetLastError() << std::endl;
        return 1;
    }

    SharedData* pSharedData = static_cast<SharedData*>(MapViewOfFile(
        hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(SharedData)));

    if (!pSharedData) {
        std::cerr << "MapViewOfFile failed: " << GetLastError() << std::endl;
        CloseHandle(hMapFile);
        return 1;
    }

    // 创建事件对象
    HANDLE hDataWrittenEvent = CreateEvent(nullptr, FALSE, FALSE, DATA_WRITTEN_EVENT_NAME);
    if (!hDataWrittenEvent) {
        std::cerr << "CreateEvent failed: " << GetLastError() << std::endl;
        UnmapViewOfFile(pSharedData);
        CloseHandle(hMapFile);
        return 1;
    }

    // 初始化串口
    const LPCTSTR port_name = _T("\\\\.\\COM18"); // 使用完整设备路径
    HANDLE hSerial = CreateFile(port_name, GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                               OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);

    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "CreateFile for COM failed: " << GetLastError() << std::endl;
        UnmapViewOfFile(pSharedData);
        CloseHandle(hMapFile);
        CloseHandle(hDataWrittenEvent);
        return 1;
    }

    // 配置串口参数
    DCB dcb = {};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hSerial, &dcb)) {
        std::cerr << "GetCommState failed: " << GetLastError() << std::endl;
        CloseHandle(hSerial);
        UnmapViewOfFile(pSharedData);
        CloseHandle(hMapFile);
        CloseHandle(hDataWrittenEvent);
        return 1;
    }

    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    dcb.fDtrControl = DTR_CONTROL_ENABLE; // 启用DTR流控制
    dcb.fRtsControl = RTS_CONTROL_ENABLE; // 启用RTS流控制

    if (!SetCommState(hSerial, &dcb)) {
        std::cerr << "SetCommState failed: " << GetLastError() << std::endl;
        CloseHandle(hSerial);
        UnmapViewOfFile(pSharedData);
        CloseHandle(hMapFile);
        CloseHandle(hDataWrittenEvent);
        return 1;
    }

    // 设置串口缓冲区
    if (!SetupComm(hSerial, 4096, 4096)) {
        std::cerr << "SetupComm failed: " << GetLastError() << std::endl;
        CloseHandle(hSerial);
        UnmapViewOfFile(pSharedData);
        CloseHandle(hMapFile);
        CloseHandle(hDataWrittenEvent);
        return 1;
    }

    // 配置超时 - 非阻塞模式
    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = MAXDWORD; // 完全非阻塞
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        std::cerr << "SetCommTimeouts failed: " << GetLastError() << std::endl;
        CloseHandle(hSerial);
        UnmapViewOfFile(pSharedData);
        CloseHandle(hMapFile);
        CloseHandle(hDataWrittenEvent);
        return 1;
    }

    // 初始化环形缓冲区和重叠IO
    CircularBuffer buffer(BUFFER_SIZE);
    OVERLAPPED overlapped = {};
    overlapped.hEvent = CreateEvent(nullptr, TRUE, FALSE, nullptr);
    if (!overlapped.hEvent) {
        std::cerr << "CreateEvent for overlapped failed: " << GetLastError() << std::endl;
        CloseHandle(hSerial);
        UnmapViewOfFile(pSharedData);
        CloseHandle(hMapFile);
        CloseHandle(hDataWrittenEvent);
        return 1;
    }

    unsigned char io_buffer[512]; // I/O缓冲区
    DWORD bytes_read = 0;
    bool pending_io = false;

    // 发起第一次异步读取
    if (!ReadFile(hSerial, io_buffer, sizeof(io_buffer), &bytes_read, &overlapped)) {
        if (GetLastError() == ERROR_IO_PENDING) {
            pending_io = true;
        } else {
            std::cerr << "Initial ReadFile failed: " << GetLastError() << std::endl;
            CloseHandle(overlapped.hEvent);
            CloseHandle(hSerial);
            UnmapViewOfFile(pSharedData);
            CloseHandle(hMapFile);
            CloseHandle(hDataWrittenEvent);
            return 1;
        }
    } else if (bytes_read > 0) {
        buffer.write(io_buffer, bytes_read);
    }

    std::cout << "System initialized. Waiting for data..." << std::endl;

    while (true) {
        DWORD wait_result = WaitForSingleObject(overlapped.hEvent, 0);

        if (wait_result == WAIT_OBJECT_0) {
            // 异步I/O完成
            if (GetOverlappedResult(hSerial, &overlapped, &bytes_read, FALSE)) {
                if (bytes_read > 0) {
                    buffer.write(io_buffer, bytes_read);
                }
            }

            // 重置事件并启动新的异步读取
            ResetEvent(overlapped.hEvent);
            pending_io = false;

            if (!ReadFile(hSerial, io_buffer, sizeof(io_buffer), nullptr, &overlapped)) {
                if (GetLastError() == ERROR_IO_PENDING) {
                    pending_io = true;
                } else {
                    std::cerr << "ReadFile failed: " << GetLastError() << std::endl;
                    break;
                }
            }
        } else if (wait_result == WAIT_TIMEOUT) {
            // 无I/O事件，处理数据
        } else {
            std::cerr << "WaitForSingleObject failed: " << GetLastError() << std::endl;
            break;
        }

        // 处理所有完整的数据块
        while (true) {



            std::vector<unsigned char> block;
            if (!buffer.read_bytes(BLOCK_SIZE, block)) break;

            // 将8位数据转换为浮点数（0-255映射到0.0-1.0）
            pSharedData->num_floats = BLOCK_SIZE;
            for (int i = 0; i < BLOCK_SIZE; i++) {
                pSharedData->data[i] = static_cast<float>(block[i]) / 255.0f;
            }

            SetEvent(hDataWrittenEvent);
            std::cout << "Processed block with " << BLOCK_SIZE << " data points" << std::endl;
        }
    }

    // 清理资源
    CancelIo(hSerial); // 取消所有未完成IO
    CloseHandle(overlapped.hEvent);
    CloseHandle(hSerial);
    UnmapViewOfFile(pSharedData);
    CloseHandle(hMapFile);
    CloseHandle(hDataWrittenEvent);

    return 0;
}