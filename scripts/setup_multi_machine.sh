#!/bin/bash
# 多机ROS通信配置脚本
# Multi-Machine ROS Communication Setup Script

set -e

echo "=========================================="
echo "多机ROS通信配置"
echo "Multi-Machine ROS Communication Setup"
echo "=========================================="

# 检测当前机器类型
HOSTNAME=$(hostname)
IP_ADDRESS=$(hostname -I | awk '{print $1}')

echo -e "\n当前机器信息:"
echo "  主机名: $HOSTNAME"
echo "  IP地址: $IP_ADDRESS"

# 询问机器类型
echo -e "\n请选择机器类型:"
echo "1) PC (主机，运行roscore和YOLOv5)"
echo "2) Jetson (小车，运行底盘和传感器)"
read -p "请输入选择 (1/2): " machine_type

if [ "$machine_type" = "1" ]; then
    # PC端配置
    echo -e "\n配置PC端（主机）..."
    
    read -p "请输入PC的IP地址 [$IP_ADDRESS]: " pc_ip
    pc_ip=${pc_ip:-$IP_ADDRESS}
    
    # 设置ROS_MASTER_URI指向自己
    export ROS_MASTER_URI=http://$pc_ip:11311
    export ROS_IP=$pc_ip
    
    echo -e "\n添加到 ~/.bashrc:"
    echo "export ROS_MASTER_URI=http://$pc_ip:11311"
    echo "export ROS_IP=$pc_ip"
    
    read -p "是否添加到 ~/.bashrc? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        # 移除旧的ROS_MASTER_URI和ROS_IP设置
        sed -i '/ROS_MASTER_URI/d' ~/.bashrc
        sed -i '/ROS_IP/d' ~/.bashrc
        
        # 添加新的设置
        echo "" >> ~/.bashrc
        echo "# Multi-machine ROS setup" >> ~/.bashrc
        echo "export ROS_MASTER_URI=http://$pc_ip:11311" >> ~/.bashrc
        echo "export ROS_IP=$pc_ip" >> ~/.bashrc
        
        echo "✓ 已添加到 ~/.bashrc"
    fi
    
    # 配置防火墙
    echo -e "\n配置防火墙..."
    if command -v ufw &> /dev/null; then
        read -p "是否开放ROS端口? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            sudo ufw allow 11311/tcp
            sudo ufw allow 11312/tcp
            echo "✓ 防火墙已配置"
        fi
    else
        echo "⚠ ufw未安装，请手动配置防火墙"
    fi
    
    echo -e "\n=========================================="
    echo "PC端配置完成！"
    echo "=========================================="
    echo -e "\n下一步:"
    echo "1. 启动roscore: roscore"
    echo "2. 启动检测节点: roslaunch navigation_stack pavs_navigation_pc_only.launch"
    echo "3. 告诉Jetson端你的IP地址: $pc_ip"
    
elif [ "$machine_type" = "2" ]; then
    # Jetson端配置
    echo -e "\n配置Jetson端（小车）..."
    
    read -p "请输入PC的IP地址: " pc_ip
    
    if [ -z "$pc_ip" ]; then
        echo "✗ PC IP地址不能为空"
        exit 1
    fi
    
    # 测试连接
    echo -e "\n测试连接到PC..."
    if ping -c 1 $pc_ip &> /dev/null; then
        echo "✓ 可以ping通PC"
    else
        echo "⚠ 无法ping通PC，请检查网络连接"
    fi
    
    # 设置ROS_MASTER_URI指向PC
    export ROS_MASTER_URI=http://$pc_ip:11311
    export ROS_IP=$IP_ADDRESS
    
    echo -e "\n添加到 ~/.bashrc:"
    echo "export ROS_MASTER_URI=http://$pc_ip:11311"
    echo "export ROS_IP=$IP_ADDRESS"
    
    read -p "是否添加到 ~/.bashrc? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        # 移除旧的ROS_MASTER_URI和ROS_IP设置
        sed -i '/ROS_MASTER_URI/d' ~/.bashrc
        sed -i '/ROS_IP/d' ~/.bashrc
        
        # 添加新的设置
        echo "" >> ~/.bashrc
        echo "# Multi-machine ROS setup" >> ~/.bashrc
        echo "export ROS_MASTER_URI=http://$pc_ip:11311" >> ~/.bashrc
        echo "export ROS_IP=$IP_ADDRESS" >> ~/.bashrc
        
        echo "✓ 已添加到 ~/.bashrc"
    fi
    
    echo -e "\n=========================================="
    echo "Jetson端配置完成！"
    echo "=========================================="
    echo -e "\n下一步:"
    echo "1. 确保PC端已启动roscore"
    echo "2. 测试连接: rostopic list"
    echo "3. 启动小车节点: roslaunch navigation_stack pavs_navigation_vehicle_only.launch"
    
else
    echo "✗ 无效的选择"
    exit 1
fi

echo -e "\n配置完成！请重新打开终端或运行: source ~/.bashrc"

