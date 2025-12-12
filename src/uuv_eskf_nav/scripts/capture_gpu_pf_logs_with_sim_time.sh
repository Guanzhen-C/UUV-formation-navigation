#!/bin/bash

# 脚本：过滤GPU-PF日志并在终端显示
# 用法：./capture_gpu_pf_logs_with_sim_time.sh

echo "正在启动terrain matching节点并过滤[GPU-PF]日志..."
echo "只在终端显示，不保存到文件"
echo "按Ctrl+C停止"

# 启动roslaunch并仅过滤显示[GPU-PF]相关的输出
roslaunch uuv_eskf_nav start_terrain_matching_gpu.launch 2>&1 | \
    stdbuf -oL -eL tr '\r' '\n' | \
    sed 's/\x1b\[[0-9;]*m//g; s/.[K//g; s/\x0f//g; s/.[2K//g' | \
    while IFS= read -r line; do
        if [[ $line == *"[GPU-PF]"* ]]; then
            # 进一步清理行内容，只保留可打印字符
            clean_line=$(echo "$line" | LC_ALL=C sed 's/[^[:print:][:space:]]//g' | sed 's/[[:cntrl:]]//g')
            # 确保输出以换行符结尾
            printf '%s\n' "$clean_line"
        fi
    done
