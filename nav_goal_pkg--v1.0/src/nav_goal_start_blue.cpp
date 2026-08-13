#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <iostream>
#include <vector>
#include <limits>

// 目标点结构体：名称 + 坐标(x,y) + 朝向四元数(z,w)
struct GoalPoint {
    std::string name;   // 目标点名称
    double x, y;        // 目标坐标（map 坐标系）
    double z, w;        // 目标朝向（四元数）
};

// 目标点列表（编号从 1 开始，与终端菜单选项对应）
// 本节点只有纯导航功能：不做二次定位和相对位移
std::vector<GoalPoint> goal_list = {
    {"出发点",  -0.006, 0.220,  0.695,  0.719},
    {"充电桩",   0.670, 2.878,  0.709,  0.705},
    {"物资获取", 3.071, 2.020, -0.697,  0.717},
    {"加工中心", 4.418, 0.347, -0.712,  0.702},
};

// 定义 Action 客户端类型
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* nav_client;

// 打印目标点选择菜单
void printMenu(){
    printf("\n======================================\n");
    printf("      请选择目标点（输入数字）\n");
    printf("======================================\n");
    for (int i = 0; i < (int)goal_list.size(); i++){
        printf("  %d - %s\n", i + 1, goal_list[i].name.c_str());
    }
    printf("  0 - 退出程序\n");
    printf("======================================\n");
}

// 读取键盘输入，返回用户选择（1~N 为目标点，0 退出，-1 输入无效）
int getChoice(){
    int choice;
    std::cin >> choice;
    if (std::cin.fail()){
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return -1;
    }
    return choice;
}

bool navToGoal(double x, double y, double z, double w){
    // 等待服务器连接成功
    ROS_INFO("等待连接 move_base 服务器...");
    nav_client->waitForServer();
    ROS_INFO("连接成功！");
    nav_client->cancelAllGoals();
    ROS_WARN("已清空所有导航任务！");
    // 构造导航目标消息
    move_base_msgs::MoveBaseGoal goal;

    // 设置坐标系为 map
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // 设置目标坐标
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    // 设置朝向
    goal.target_pose.pose.orientation.z = z;
    goal.target_pose.pose.orientation.w = w;
    // 发送目标点
    ROS_INFO("发送导航目标...");
    nav_client->sendGoal(goal);

    // 循环监听导航状态
    ros::Rate rate(5);
    while (ros::ok())
    {
        actionlib::SimpleClientGoalState state = nav_client->getState();
        std::string state_str = state.toString();

        // 实时打印状态
        ROS_INFO("当前导航状态：%s", state_str.c_str());

        // 导航成功
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("导航成功：已到达目标点！");
            return true;
        }
        // 导航失败（内部错误/障碍物/无法规划）
        else if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_ERROR("导航失败：无法到达目标！");
            return false;
        }
        // 任务被取消
        else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
        {
            ROS_WARN("导航任务已被取消！");
            return false;
        }
        // 任务被拒绝
        else if (state == actionlib::SimpleClientGoalState::REJECTED)
        {
            ROS_ERROR("导航目标被服务器拒绝！");
            return false;
        }
        // 导航超时
        else if (state == actionlib::SimpleClientGoalState::LOST)
        {
            ROS_ERROR("导航连接丢失！");
            return false;
        }
        rate.sleep();
    }
    // ROS 退出
    return false;
}

int main(int argc, char** argv) {
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "auto_nav_start_node");
    ros::NodeHandle nh;

    // 初始化 Action 客户端
    nav_client = new MoveBaseClient("move_base", true);

    int choice = -1;
    while (ros::ok()){
        // 打印菜单，等待用户选择目标点
        printMenu();
        printf("请输入编号：");
        fflush(stdout);
        choice = getChoice();

        if (choice == 0){
            ROS_INFO("程序退出！");
            break;
        }
        if (choice == -1){
            ROS_ERROR("输入无效，请重新输入！");
            continue;
        }
        if (choice < 1 || choice > (int)goal_list.size()){
            ROS_ERROR("编号超出范围，请输入 1~%d 或 0 退出！", (int)goal_list.size());
            continue;
        }

        // 纯导航：不做二次定位和相对位移
        GoalPoint& goal = goal_list[choice - 1];
        ROS_INFO("已选择目标点：%s", goal.name.c_str());

        if (!navToGoal(goal.x, goal.y, goal.z, goal.w)) {
            ROS_ERROR("导航失败：%s", goal.name.c_str());
            continue;
        }
        ROS_INFO("任务完成：%s", goal.name.c_str());
    }

    delete nav_client;
    return 0;
}
