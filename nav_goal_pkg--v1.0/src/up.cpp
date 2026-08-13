#include <ros/ros.h>
#include "relative_move/SetRelativeMove.h"
#include <iostream>

// 向前移动距离（米，相对机器人坐标系，前方向为正x）
const float FORWARD_MOVE_X = 0.10;

// 创建服务客户端
ros::ServiceClient relmove_client;

// 相对位移（向前）：x 为相对机器人坐标系的前后距离，正值向前
bool set_relmove(float x, float y, float theta){
    // 等待服务上线
    ROS_INFO("等待服务 /relative_move 启动...");
    relmove_client.waitForExistence();
    ROS_INFO("服务已连接！");
    // 定义服务消息
    relative_move::SetRelativeMove srv;

    // 填充请求数据
    srv.request.goal.x = x;
    srv.request.goal.y = y;
    srv.request.goal.theta = theta;
    srv.request.global_frame = "odom";
    // 发送请求
    if (relmove_client.call(srv)){
        if (srv.response.success){
            ROS_INFO("移动成功：%s", srv.response.message.c_str());
            return 1;
        }else{
            ROS_ERROR("移动失败：%s", srv.response.message.c_str());
            return 0;
        }
    }else{
        ROS_ERROR("服务调用失败！");
        return 0;
    }
}

int main(int argc, char** argv) {
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "forward_move_node");
    ros::NodeHandle nh;

    // 初始化服务客户端
    relmove_client = nh.serviceClient<relative_move::SetRelativeMove>("/relative_move");

    // 执行向前移动 0.10m
    ROS_INFO("开始向前移动 %.2f m...", FORWARD_MOVE_X);
    if (!set_relmove(FORWARD_MOVE_X, 0, 0)) {
        ROS_ERROR("向前移动失败！");
        return -1;
    }

    ROS_INFO("向前移动完成！");
    return 0;
}
