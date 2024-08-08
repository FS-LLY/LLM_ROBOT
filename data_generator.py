import subprocess
import matplotlib.pyplot as plt
import numpy as np
import time
import json
import os

# 替换为你的C++程序的路径
cpp_program_path = "cmake-build-debug/base/test_every_joint"

# 启动C++程序并创建一个到其标准输入的管道
p = subprocess.Popen(cpp_program_path, stdin=subprocess.PIPE)

time_cnt = 0

# 文件路径
file_path = "simulate_datasets/order.txt"
time_span = 110  # 假设这里是 100 ms 时间延迟，具体的需要按收包实际时间差考虑
total_time = 250
last_data = [0, -40.0 * np.pi / 180.0, 0, 90 * np.pi / 180, 0, 40 / 180 * np.pi, 0]
data_a4_last = []
data_a4_angle = []
is_gripper_closed = False

finish_data = []

def file_exists(directory, filename):
    file_path = os.path.join(directory, filename)
    return os.path.exists(file_path) and os.path.isfile(file_path)

def handle_each_file():
    try:
        with open(file_path, "r") as file:
            for line in file:
                line = line.replace("None", "null")
                time_cnt = time_cnt + 1
                if time_cnt > 40:
                    time_span = 30
                    print("Move faster")
                try:
                    data = json.loads(line)  # 解析JSON
                    if data["joint_positions"][3]:
                        data_a4_angle.append(data["joint_positions"][2])

                    is_gripper_closed = bool(data["joint_positions"][0])

                    diff_data = []
                    # 计算两个变量的差值，用于预测下一帧的走向
                    if data["joint_positions"][0]:
                        for j in range(7):
                            diff_data.append(
                                (data["joint_positions"][j] - last_data[j]) / time_span
                            )
                        last_data = data["joint_positions"]

                    for i in range(time_span):
                        local_data = [0, 0, 0, 0, 0, 0, 0]
                        if data["joint_positions"][0]:
                            for j in range(7):
                                local_data[j] = data["joint_positions"][
                                    j
                                ]  # todo: 这里可能是硬拷贝的问题，明天有空查一下
                                local_data[j] = local_data[j] + diff_data[j] * i
                                # if j == 3:
                                # data_a4_last.append(local_data[3])
                        # 确保每一行都是有效的JSON格式
                        if data["joint_positions"][0]:
                            local_datas = {
                                "joint_positions": local_data,
                                "joint_velocities": [0, 0, 0, 0, 0, 0, 0],
                                "joint_efforts": is_gripper_closed,
                            }
                            finish_data = local_datas
                        else:
                            local_datas = {
                                "joint_positions": last_data,
                                "joint_velocities": [0, 0, 0, 0, 0, 0, 0],
                                "joint_efforts": is_gripper_closed,
                            }
                            finish_data = local_datas
                        json_string = json.dumps(
                            local_datas
                        )  # 将解析后的JSON重新编码为字符串
                        data_a4_last.append(local_data[2] * 180 / np.pi)
                        p.stdin.write(
                            json_string.encode() + b"\n"
                        )  # 发送JSON字符串给C++程序
                        p.stdin.flush()  # 确保数据被发送
                        time.sleep(0.001)  # 以1000Hz的频率发送数据
                        # if not next(file):
                        #     break

                except json.JSONDecodeError as e:
                    print(f"JSON解析错误: {e}")

            while 1:
                json_finish_string = json.dumps(finish_data)
                p.stdin.write(json_finish_string.encode() + b"\n")
                p.stdin.flush()
                time.sleep(0.001)
                if file_exists('simulate_datasets','order.txt'): 
                    return

            # plt.scatter(t2, data_a4_angle)
    except KeyboardInterrupt:
        p.terminate()  # 用户中断脚本时终止C++程序

file_cnt = 0

while not file_exists('simulate_datasets','order0.txt'):
    print('waiting...')

time.sleep(4)

while 1:
    file_name = 'order' + str(file_cnt) + '.txt'
    # file_name = 'order0.txt'
    if (file_exists('simulate_datasets','order1.txt') and file_cnt == 0) or (file_exists('simulate_datasets',file_name) and file_cnt > 0):
        try:
            print("Processing file " + str(file_cnt))
            with open('simulate_datasets/' + file_name, "r") as file:
                for line in file:
                    line = line.replace("None", "null")
                    time_cnt = time_cnt + 1
                    if time_cnt > 10:
                        time_span = 15
                        # print("Move faster")
                    try:
                        data = json.loads(line)  # 解析JSON
                        if data["joint_positions"][3]:
                            data_a4_angle.append(data["joint_positions"][2])

                        is_gripper_closed = bool(data["joint_positions"][0])

                        diff_data = []
                        # 计算两个变量的差值，用于预测下一帧的走向
                        if data["joint_positions"][0]:
                            for j in range(7):
                                diff_data.append(
                                    (data["joint_positions"][j] - last_data[j]) / time_span
                                )
                            last_data = data["joint_positions"]

                        for i in range(time_span):
                            local_data = [0, 0, 0, 0, 0, 0, 0]
                            if data["joint_positions"][0]:
                                for j in range(7):
                                    local_data[j] = data["joint_positions"][
                                        j
                                    ]  # todo: 这里可能是硬拷贝的问题，明天有空查一下
                                    local_data[j] = local_data[j] + diff_data[j] * i
                                    # if j == 3:
                                    # data_a4_last.append(local_data[3])
                            # 确保每一行都是有效的JSON格式
                            if data["joint_positions"][0]:
                                local_datas = {
                                    "joint_positions": local_data,
                                    "joint_velocities": [0, 0, 0, 0, 0, 0, 0],
                                    "joint_efforts": is_gripper_closed,
                                }
                                finish_data = local_datas
                            else:
                                local_datas = {
                                    "joint_positions": last_data,
                                    "joint_velocities": [0, 0, 0, 0, 0, 0, 0],
                                    "joint_efforts": is_gripper_closed,
                                }
                                finish_data = local_datas
                            json_string = json.dumps(
                                local_datas
                            )  # 将解析后的JSON重新编码为字符串
                            data_a4_last.append(local_data[2] * 180 / np.pi)
                            p.stdin.write(
                                json_string.encode() + b"\n"
                            )  # 发送JSON字符串给C++程序
                            p.stdin.flush()  # 确保数据被发送
                            time.sleep(0.001)  # 以1000Hz的频率发送数据
                            # if not next(file):
                            #     break

                    except json.JSONDecodeError as e:
                        print(f"JSON解析错误: {e}")

                while 1:
                    json_finish_string = json.dumps(finish_data)
                    p.stdin.write(json_finish_string.encode() + b"\n")
                    p.stdin.flush()
                    time.sleep(0.001)
                    if file_exists('simulate_datasets','order' + str(file_cnt + 1)+'.txt'):
                        break

        except KeyboardInterrupt:
            p.terminate()  # 用户中断脚本时终止C++程序

        file_cnt = file_cnt + 1
        print("Finish file " + str(file_cnt))
    else:
        break
