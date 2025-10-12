import pygame
import sys
import ctypes
import ctypes.wintypes
import win32event
import win32api

# 初始化Pygame
pygame.init()

# 示波器参数
WIDTH, HEIGHT = 800, 600
BG_COLOR = (0, 0, 0)
GRID_COLOR = (30, 30, 30)
WAVE_COLOR = (0, 255, 0)
TEXT_COLOR = (200, 200, 200)

# 创建窗口
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Python 示波器")
font = pygame.font.SysFont('consolas', 16)

# 共享内存和事件名称
SHARED_MEM_NAME = "CPP_PYTHON_SHARE"
DATA_WRITTEN_EVENT_NAME = "CPP_PYTHON_EVENT"

# 定义共享数据结构 (与C++端匹配)
class SharedData(ctypes.Structure):
    _fields_ = [
        ("num_floats", ctypes.c_int),
        ("data", ctypes.c_float * 64)  # 64个浮点数
    ]

# 定义Windows常量
FILE_MAP_ALL_ACCESS = 0xF001F
EVENT_ALL_ACCESS = 0x1F0003
WAIT_OBJECT_0 = 0

# 使用ctypes定义Windows API函数
kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)

# 定义函数原型
OpenFileMapping = kernel32.OpenFileMappingW
OpenFileMapping.argtypes = [
    ctypes.wintypes.DWORD,  # dwDesiredAccess
    ctypes.wintypes.BOOL,   # bInheritHandle
    ctypes.wintypes.LPCWSTR # lpName
]
OpenFileMapping.restype = ctypes.wintypes.HANDLE

MapViewOfFile = kernel32.MapViewOfFile
MapViewOfFile.argtypes = [
    ctypes.wintypes.HANDLE, # hFileMappingObject
    ctypes.wintypes.DWORD,  # dwDesiredAccess
    ctypes.wintypes.DWORD,  # dwFileOffsetHigh
    ctypes.wintypes.DWORD,  # dwFileOffsetLow
    ctypes.c_size_t         # dwNumberOfBytesToMap
]
MapViewOfFile.restype = ctypes.c_void_p

UnmapViewOfFile = kernel32.UnmapViewOfFile
UnmapViewOfFile.argtypes = [ctypes.c_void_p]
UnmapViewOfFile.restype = ctypes.wintypes.BOOL

CloseHandle = kernel32.CloseHandle
CloseHandle.argtypes = [ctypes.wintypes.HANDLE]
CloseHandle.restype = ctypes.wintypes.BOOL

# 打开共享内存
try:
    # 打开文件映射对象
    h_map = OpenFileMapping(
        FILE_MAP_ALL_ACCESS,  # 读写权限
        False,                # 不继承
        SHARED_MEM_NAME       # 共享内存名称
    )
    
    if not h_map:
        raise ctypes.WinError(ctypes.get_last_error())
    
    # 映射共享内存视图
    addr = MapViewOfFile(
        h_map,
        FILE_MAP_ALL_ACCESS,
        0,  # 文件偏移高32位
        0,  # 文件偏移低32位
        ctypes.sizeof(SharedData)  # 映射大小
    )
    
    if not addr:
        raise ctypes.WinError(ctypes.get_last_error())
    
    # 将共享内存映射到结构体
    shared_data = ctypes.cast(addr, ctypes.POINTER(SharedData)).contents
    
except Exception as e:
    print(f"共享内存错误: {e}")
    sys.exit()

# 打开事件对象
try:
    # 使用win32event打开事件对象
    h_event = win32event.OpenEvent(
        EVENT_ALL_ACCESS,  # 访问权限
        False,             # 不继承
        DATA_WRITTEN_EVENT_NAME  # 事件名称
    )
    
    if h_event is None:
        raise Exception(f"无法打开事件对象: {DATA_WRITTEN_EVENT_NAME}")
    
except Exception as e:
    print(f"事件对象错误: {e}")
    UnmapViewOfFile(addr)
    CloseHandle(h_map)
    sys.exit()

# 数据缓冲区
data_points = [0] * WIDTH
current_index = 0

# 绘制网格
def draw_grid():
    # 水平网格
    for y in range(0, HEIGHT, 20):
        pygame.draw.line(screen, GRID_COLOR, (0, y), (WIDTH, y), 1)
    
    # 垂直网格
    for x in range(0, WIDTH, 20):
        pygame.draw.line(screen, GRID_COLOR, (x, 0), (x, HEIGHT), 1)
    
    # 中心线
    pygame.draw.line(screen, (100, 100, 100), (0, HEIGHT//2), (WIDTH, HEIGHT//2), 2)

# 主循环
clock = pygame.time.Clock()
running = True

print("系统已初始化。等待波形数据...")

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
    
    # 检查事件是否触发
    event_state = win32event.WaitForSingleObject(h_event, 0)
    
    if event_state == WAIT_OBJECT_0:
        # 事件已触发，读取新数据
        num_points = min(shared_data.num_floats, 64)
        
        # 将浮点数转换为0-255的整数
        for i in range(num_points):
            # 将0.0-1.0的浮点数映射到0-255
            value = int(shared_data.data[i] * 255)
            value = max(0, min(255, value))  # 确保在范围内
            
            # 更新数据点
            data_points[current_index] = value
            current_index = (current_index + 1) % WIDTH
    
    # 清屏
    screen.fill(BG_COLOR)
    
    # 绘制网格
    draw_grid()
    
    # 绘制波形
    points = []
    for i in range(WIDTH):
        idx = (current_index + i) % WIDTH
        # 将数据值转换为屏幕坐标 (0-255 -> 0-HEIGHT)
        y = HEIGHT - int((data_points[idx] / 255) * HEIGHT)
        points.append((i, y))
    
    if len(points) > 1:
        pygame.draw.lines(screen, WAVE_COLOR, False, points, 2)
    
    # 显示信息
    fps = int(clock.get_fps())
    info_text = f"共享内存: {SHARED_MEM_NAME} | FPS: {fps} | 最新值: {data_points[(current_index - 1) % WIDTH]}"
    text_surface = font.render(info_text, True, TEXT_COLOR)
    screen.blit(text_surface, (10, 10))
    
    # 更新显示
    pygame.display.flip()
    clock.tick(60)  # 限制帧率

# 清理
UnmapViewOfFile(addr)
CloseHandle(h_map)
win32api.CloseHandle(h_event)
pygame.quit()
sys.exit()