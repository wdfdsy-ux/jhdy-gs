#include <ros/ros.h>
#include "relative_move/SetRelativeMove.h"
#include <iostream>

// 后撤距离（米，相对机器人坐标系，负值表示后退）
const float BACK_OFF_X = -0.20;

// 创建服务客户端
ros::ServiceClient relmove_client;

// 相对位移（后撤）：x 为相对机器人坐标系的前后距离，负值后退
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
    ros::init(argc, argv, "back_off_node");
    ros::NodeHandle nh;

    // 初始化服务客户端
    relmove_client = nh.serviceClient<relative_move::SetRelativeMove>("/relative_move");

    // 执行后撤 0.20m
    ROS_INFO("开始后撤 %.2f m...", BACK_OFF_X);
    if (!set_relmove(BACK_OFF_X, 0, 0)) {
        ROS_ERROR("后撤失败！");
        return -1;
    }

    ROS_INFO("后撤完成！");
    return 0;
}
