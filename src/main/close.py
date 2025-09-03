import psutil

# 设置包含关键词的列表，如果进程命令行包含这些关键词，则关闭该进程
keywords_to_close = ['navacation', 't_dis', 'obstacle', 'start.launch',  'xunxian']

# 遍历所有进程
for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
    try:
        cmdline = proc.cmdline()
    except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
        continue
    
    if any(keyword in ' '.join(cmdline) for keyword in keywords_to_close):
        print(f"Closing process: {proc.pid} - {proc.name()} - {' '.join(cmdline)}")
        proc.terminate()  # 可以使用 proc.kill() 强制终止进程
