import os

def delete_files(folder_path):
    for root, dirs, files in os.walk(folder_path):
        for file in files:
            file_path = os.path.join(root, file)
            os.remove(file_path)
            print(f"Deleted file: {file_path}")

# 指定要删除文件的文件夹路径
folder_path = './image_result'

# 调用函数删除文件
delete_files(folder_path)