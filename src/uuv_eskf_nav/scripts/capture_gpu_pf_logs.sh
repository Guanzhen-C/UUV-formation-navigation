#!/bin/bash

# 脚本：捕获GPU-PF日志并添加时间戳
# 用法：./capture_gpu_pf_logs.sh <输出文件名>

OUTPUT_FILE=${1:-"gpu_pf_logs.txt"}

echo "正在启动terrain matching节点并捕获[GPU-PF]日志..."
echo "输出将保存到: $OUTPUT_FILE"
echo "按Ctrl+C停止捕获"

# 启动roslaunch并将[GPU-PF]相关的输出保存到文件，包含时间戳
roslaunch uuv_eskf_nav start_terrain_matching_gpu.launch 2>&1 | \
    while IFS= read -r line; do
        if [[ $line == *"[GPU-PF]"* ]]; then
            echo "$(date '+%Y-%m-%d %H:%M:%S') $line" >> "$OUTPUT_FILE"
            echo "$line"  # 同时输出到终端以便查看
        fi
    done