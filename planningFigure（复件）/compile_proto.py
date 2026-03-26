import os
import subprocess

# 指定需要编译的.proto文件所在的根目录
root_dirs = ["../planningFigure"]


# 指定protoc编译器的路径，如果你已经将protoc加入环境变量，则不需要指定
protoc_path = "protoc"

# 递归遍历目录，编译所有的.proto文件
for root_dir in root_dirs:
    for subdir, dirs, files in os.walk(root_dir):
        for file in files:
            if file.endswith(".proto"):
                # 构建完整的.proto文件路径
                proto_file = os.path.join(subdir, file)
                # 构建输出目录
                output = os.path.join(subdir, "./")
                # 构建protoc命令
                command = [
                    protoc_path, 
                    # "--python_out={}".format(output),
                    "--cpp_out={}".format(output),
                    "--proto_path={}".format(subdir),  # 添加proto_path参数
                    proto_file
                ]
                # 执行protoc命令
                print("Compiling:", proto_file)
                subprocess.run(command, check=True)

print("All .proto files have been compiled.")
 
