/**
 * @file udp_receiver_raw.cpp
 * @brief 一个用于接收FPGA高速ADC原始数据的UDP服务器程序 (Windows平台)
 * 并通过共享内存将数据传递给其他进程。
 *
 * 功能:
 * 1. 初始化Winsock网络库。
 * 2. 创建一个UDP套接字并绑定到指定IP和端口。
 * 3. 创建一个用于进程间通信的共享内存区域。
 * 4. 循环接收UDP数据包，将整个负载作为ADC原始数据。
 * 5. 将最新的ADC样本数据写入共享内存。
 * 6. 打印接收统计信息。
 * 7. 在程序退出时正确清理资源。
 *
 * 编译说明 (使用g++):
 * g++ -o udp_receiver_raw.exe udp_receiver_raw.cpp -lws2_32
 */

#include <iostream>
#include <vector>
#include <cstdint>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <chrono>
#include <string>

// --- 配置参数 ---
constexpr const char* LISTEN_IP = "0.0.0.0";      // 监听所有网络接口
constexpr int LISTEN_PORT = 6102;                 // 监听端口，与FPGA的dst_port匹配
constexpr int MAX_PACKET_SIZE = 2048;             // 接收缓冲区的最大大小
constexpr int RECV_TIMEOUT_MS = 5000;             // 5秒接收超时

// --- 共享内存配置 ---
constexpr const TCHAR* SHARED_MEM_NAME = TEXT("FPGA_ADC_DATA"); // 共享内存名称
constexpr int SHARED_MEM_SIZE = 2048;      // 共享内存大小，应大于等于最大ADC数据长度

// 获取Winsock错误信息的辅助函数
std::string get_socket_error() {
    int error_code = WSAGetLastError();
    char buffer[256];
    FormatMessageA(
        FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL, error_code, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        buffer, sizeof(buffer), NULL
    );
    return std::string(buffer) + " (Error Code: " + std::to_string(error_code) + ")";
}

int main() {
    // 1. 初始化 Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed: " << get_socket_error() << std::endl;
        return 1;
    }

    // 2. 创建共享内存
    HANDLE hMapFile = CreateFileMapping(
    INVALID_HANDLE_VALUE,  // 使用系统分页文件
    NULL,                  // 默认安全属性
    PAGE_READWRITE,        // 可读可写
    0,                     // 内存大小高32位
    SHARED_MEM_SIZE,       // 内存大小低32位
    SHARED_MEM_NAME);      // 共享内存名称

    if (hMapFile == NULL) {
        std::cerr << "Could not create file mapping object: " << GetLastError() << std::endl;
        WSACleanup();
        return 1;
    }

    uint8_t* pSharedMem = (uint8_t*)MapViewOfFile(
        hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, SHARED_MEM_SIZE);

    if (pSharedMem == NULL) {
        std::cerr << "Could not map view of file: " << GetLastError() << std::endl;
        CloseHandle(hMapFile);
        WSACleanup();
        return 1;
    }
    std::cout << "Shared memory '" << SHARED_MEM_NAME << "' created successfully." << std::endl;

    // 3. 创建 UDP 套接字
    SOCKET sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd == INVALID_SOCKET) {
        std::cerr << "Failed to create socket: " << get_socket_error() << std::endl;
        WSACleanup();
        return 1;
    }

    // 4. 设置接收超时
    DWORD timeout = RECV_TIMEOUT_MS;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));

    // 5. 绑定套接字到IP和端口
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(LISTEN_PORT);
    server_addr.sin_addr.s_addr = inet_addr(LISTEN_IP);
    
    if (bind(sockfd, (sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        std::cerr << "Bind failed: " << get_socket_error() << std::endl;
        closesocket(sockfd);
        WSACleanup();
        return 1;
    }

    std::cout << "Listening on " << LISTEN_IP << ":" << LISTEN_PORT << " for raw ADC data..." << std::endl;

    // 6. 准备接收循环
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE);
    long long total_bytes_received = 0;
    auto start_time = std::chrono::steady_clock::now();

    // 7. 主接收循环
    while (true) {
        sockaddr_in client_addr{};
        int client_addr_len = sizeof(client_addr);

        int bytes_received = recvfrom(
            sockfd, (char*)buffer.data(), buffer.size(), 0,
            (sockaddr*)&client_addr, &client_addr_len
        );

        if (bytes_received == SOCKET_ERROR) {
            if (WSAGetLastError() == WSAETIMEDOUT) {
                std::cout << "Receive timeout, no data from FPGA. Waiting..." << std::endl;
            } else {
                std::cerr << "Receive error: " << get_socket_error() << std::endl;
            }
            continue;
        }

        if (bytes_received > 0) {
            // --- 整个负载都是ADC数据 ---
            const uint8_t* adc_samples = buffer.data();
            size_t sample_count = bytes_received;

            // --- 写入共享内存 ---
            // 结构: 前2个字节存储采样点数，后面跟着ADC数据
            if (sample_count + 2 <= SHARED_MEM_SIZE) {
                uint16_t count_net_order = htons(static_cast<uint16_t>(sample_count));
                memcpy(pSharedMem, &count_net_order, 2);
                memcpy(pSharedMem + 2, adc_samples, sample_count);
            } else {
                 std::cerr << "Warning: ADC data size (" << sample_count
                           << ") exceeds shared memory capacity." << std::endl;
            }
        
            // --- 统计信息 ---
            total_bytes_received += bytes_received;
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();

            if (elapsed_ms >= 1000) {
                double data_rate_mbps = (total_bytes_received * 8.0) / (elapsed_ms / 1000.0) / 1000000.0;
                printf("Rate: %-8.2f Mbps | Last Packet Samples: %-5zu\n",
                       data_rate_mbps, sample_count);
                total_bytes_received = 0;
                start_time = current_time;
            }
        }
    }

    // 8. 清理资源
    UnmapViewOfFile(pSharedMem);
    CloseHandle(hMapFile);
    closesocket(sockfd);
    WSACleanup();
    return 0;
}