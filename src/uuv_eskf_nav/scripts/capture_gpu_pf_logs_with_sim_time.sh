#!/bin/bash

# 脚本：捕获GPU-PF日志并添加仿真时间戳，每1000条数据保存在一个文件中
# 用法：./capture_gpu_pf_logs_with_sim_time.sh

OUTPUT_DIR="gpu_pf_logs"
mkdir -p "$OUTPUT_DIR"

# 获取当前时间作为基准
BASE_TIME=$(date +%Y%m%d_%H%M%S)

echo "正在启动terrain matching节点并捕获[GPU-PF]日志..."
echo "输出将保存到: $OUTPUT_DIR 目录下"
echo "按Ctrl+C停止捕获"

# 初始化变量
current_file_num=1
current_line_count=0
current_file="${OUTPUT_DIR}/gpu_pf_logs_${BASE_TIME}_${current_file_num}.txt"

# 函数：检查并切换到新文件
switch_to_new_file() {
    current_file_num=$((current_file_num + 1))
    current_file="${OUTPUT_DIR}/gpu_pf_logs_${BASE_TIME}_${current_file_num}.txt"
    current_line_count=0
    echo "切换到新文件: $current_file"
}

# 获取仿真时间的函数
get_sim_time() {
    # 从ROS中获取仿真时间
    ros_time=$(rostopic echo -n 1 /clock 2>/dev/null | grep -oP 'secs: \K\d+' 2>/dev/null || echo "0")
    if [ "$ros_time" = "0" ]; then
        # 如果无法获取仿真时间，则使用系统时间
        ros_time=$(date +%s)
    fi
    echo "$ros_time"
}

# 启动roslaunch并将[GPU-PF]相关的输出保存到文件，包含仿真时间戳
roslaunch uuv_eskf_nav start_terrain_matching_gpu.launch 2>&1 | \
    while IFS= read -r line; do
        if [[ $line == *"[GPU-PF]"* ]]; then
            # 获取仿真时间
            sim_time=$(get_sim_time)
            # 输出带时间戳的行
            echo "$sim_time $line" >> "$current_file"
            echo "$line"  # 同时输出到终端以便查看
            
            current_line_count=$((current_line_count + 1))
            
            # 检查是否达到1000条
            if [ $current_line_count -ge 1000 ]; then
                switch_to_new_file
            fi
        fi
    done
