#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "arm_controller/move.h"
#include "std_srvs/Empty.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <limits>

// 机械臂移动服务客户端
ros::ServiceClient armmove_client;
// 吸盘控制服务客户端（/swiftpro/on 开启，/swiftpro/off 关闭）
ros::ServiceClient pick_client;
ros::ServiceClient put_client;
// AR 标签坐标发布器（识别到 AR 码后发出其 TF 坐标）
ros::Publisher ar_pose_pub;

// 观测点坐标（机械臂坐标系，单位 mm）
const float OBS_POINT_X = 60.0;
const float OBS_POINT_Y = -165.0;
const float OBS_POINT_Z = 85.0;

// 安全点坐标
const float SAFE_POINT_X = 150.0;
const float SAFE_POINT_Y = 0.0;
const float SAFE_POINT_Z = 120.0;

// TF 坐标系：父坐标系（机械臂底座）/ 子坐标系（AR 标签，由 ar_track_alvar 发布）
const std::string BASE_FRAME = "Base";
const std::string AR_TAG_FRAME_PREFIX = "ar_marker_";
// 扫描识别时长（秒）
const double SCAN_DURATION = 5.0;

// 平移点定义：名称 + 基准AR码ID + x方向偏移
// 2号/4号由AR码100换算（x+106 / x+50），1号/3号由AR码101换算（x+106 / x+50）
// 计算规则：平移点坐标 = (AR码TF的x + x偏移, AR码TF的y)
struct ShiftDef {
    std::string name;
    int ar_id;
    float x_offset;
};

// 平移点定义列表
std::vector<ShiftDef> shift_defs = {
    {"1号点", 101, 106},
    {"2号点", 100, 106},
    {"3号点", 101,  50},
    {"4号点", 100,  50},
};

// 计算出的平移点（运行时根据AR码TF坐标填充）
struct PointXY {
    std::string name;
    float x, y;
};

// 放置点结构体：名称 + 坐标(x,y,z)
struct PlacePoint {
    std::string name;
    float x, y, z;
};

// 放置点列表（编号从 1 开始，与终端菜单选项对应）
std::vector<PlacePoint> place_points = {
    {"放置点1", 107.0, 115.0, 42.0},
    {"放置点2", 107.0, 185.0, 42.0},
};

// 抓取参数
const float GRAB_DOWN_Z = 40.0;         // 吸取高度：从85mm观测高度下降45mm
const float PLACE_LIFT_OFFSET = 30.0;   // 放置后出刀上升距离（防撞）

// 扫描过程中识别到的所有 AR 码 ID（自动去重、按 ID 升序排列）
std::set<int> detected_ids;

bool arm_move(float x, float y, float z)
{
    ROS_INFO("等待服务 /goto_position 启动...");
    armmove_client.waitForExistence();
    ROS_INFO("服务已连接！");
    arm_controller::move srv;
    srv.request.pose.position.x = x;
    srv.request.pose.position.y = y;
    srv.request.pose.position.z = z;

    if (armmove_client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("机械臂移动成功：目标(%.2f, %.2f, %.2f)", x, y, z);
            return true;
        } else {
            ROS_ERROR("机械臂移动失败：%s", srv.response.message.c_str());
            return false;
        }
    } else {
        ROS_ERROR("机械臂服务调用失败！");
        return false;
    }
}

void safe_retract()
{
    ROS_WARN("任务异常，正在收回机械臂至安全点...");
    arm_move(SAFE_POINT_X, SAFE_POINT_Y, SAFE_POINT_Z);
}

int getChoice();   // 前置声明（定义见下方）

// 吸盘开关：true 开启，false 关闭
void set_pump(bool state)
{
    ROS_INFO("等待抓取服务启动...");
    pick_client.waitForExistence();
    put_client.waitForExistence();
    ROS_INFO("服务已连接！");
    std_srvs::Empty srv;
    if (state) {
        pick_client.call(srv);
        ROS_INFO("吸盘已开启");
    } else {
        put_client.call(srv);
        ROS_INFO("吸盘已关闭");
    }
}

// 抓取功能（在平移菜单中开启，作用于当前点）：
// 下降至z=40吸取 → 吸盘开启 → 升至85mm → 玩家选择放置点 → 平移到放置点上方
// → 下降至放置高度 → 关闭吸盘放置 → 出刀上升30mm
void do_grab(float& cur_x, float& cur_y)
{
    ROS_INFO("===== 抓取功能开始 (%.2f, %.2f) =====", cur_x, cur_y);

    // 第一步：下降45mm至z=40，准备吸取
    ROS_INFO("===== 下降至吸取高度 (%.2f, %.2f, %.0f) =====", cur_x, cur_y, GRAB_DOWN_Z);
    if (!arm_move(cur_x, cur_y, GRAB_DOWN_Z)) {
        ROS_ERROR("下降失败，抓取中止！");
        return;
    }
    ros::Duration(1.0).sleep();

    // 第二步：开启吸盘，吸取物件
    ROS_INFO("===== 开启吸盘，吸取物件 =====");
    set_pump(true);

    // 第三步：上升至85mm，在此高度进行转移
    ROS_INFO("===== 上升至转移高度 (%.2f, %.2f, %.0f) =====", cur_x, cur_y, OBS_POINT_Z);
    if (!arm_move(cur_x, cur_y, OBS_POINT_Z)) {
        ROS_ERROR("上升失败，抓取中止！");
        return;
    }

    // 玩家通过键盘选择放置点
    int pc = -1;
    while (pc < 1 || pc > (int)place_points.size()) {
        printf("\n======================================\n");
        printf("      请选择放置点\n");
        printf("======================================\n");
        for (int i = 0; i < (int)place_points.size(); i++) {
            printf("  %d - %s (%.1f, %.1f, %.1f)\n", i + 1,
                   place_points[i].name.c_str(),
                   place_points[i].x, place_points[i].y, place_points[i].z);
        }
        printf("请输入编号：");
        fflush(stdout);
        pc = getChoice();
        if (pc < 1 || pc > (int)place_points.size()) {
            ROS_ERROR("输入无效，请重新选择！");
        }
    }
    PlacePoint& pp = place_points[pc - 1];

    // 第四步：在85mm高度平移到放置点上方，再下降放置
    ROS_INFO("===== 平移到 %s 上方 (%.1f, %.1f, %.0f) =====",
             pp.name.c_str(), pp.x, pp.y, OBS_POINT_Z);
    if (!arm_move(pp.x, pp.y, OBS_POINT_Z)) {
        ROS_ERROR("平移到放置点失败，抓取中止！");
        return;
    }

    ROS_INFO("===== 下降至放置高度 (%.1f, %.1f, %.1f) =====", pp.x, pp.y, pp.z);
    if (!arm_move(pp.x, pp.y, pp.z)) {
        ROS_ERROR("下降放置失败，抓取中止！");
        return;
    }
    ros::Duration(1.0).sleep();

    // 关闭吸盘，放置物块
    ROS_INFO("===== 关闭吸盘，放置物块 =====");
    set_pump(false);
    ros::Duration(1.0).sleep();

    // 出刀：上升30mm防撞，并记录当前位置
    float lift_z = pp.z + PLACE_LIFT_OFFSET;
    ROS_INFO("===== 出刀，上升至 (%.1f, %.1f, %.1f) =====", pp.x, pp.y, lift_z);
    if (!arm_move(pp.x, pp.y, lift_z)) {
        ROS_WARN("出刀失败，请手动检查机械臂！");
    }
    cur_x = pp.x;
    cur_y = pp.y;

    ROS_INFO("===== 抓取放置完成！ =====");
}

// AR 标签订阅回调：收集视野内所有 AR 码 ID（本阶段不涉及任何抓取）
void arMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& markers)
{
    for (const auto& marker : markers->markers) {
        detected_ids.insert(marker.id);
        ROS_INFO("识别到AR码 ID=%d（当前共 %zu 个）", marker.id, detected_ids.size());
    }
}

// 扫描固定时长，收集视野内的 AR 码
void scan_ar_codes()
{
    detected_ids.clear();
    ROS_INFO("开始扫描AR码...（持续 %.0f 秒）", SCAN_DURATION);
    ros::Time scan_start = ros::Time::now();
    ros::Rate rate(10);
    while (ros::ok() && (ros::Time::now() - scan_start).toSec() < SCAN_DURATION) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("扫描结束，共识别到 %zu 个AR码", detected_ids.size());
}

// 读取键盘输入，返回用户选择（-1 输入无效）
int getChoice()
{
    int choice;
    std::cin >> choice;
    if (std::cin.fail()) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return -1;
    }
    return choice;
}

// 根据AR码TF坐标计算平移点：平移点 = (AR码TF的x + x偏移, AR码TF的y)
// 只计算识别到的AR码（100/101）对应的平移点，返回是否至少算出一个点
bool compute_shift_points(tf::TransformListener& listener, std::vector<PointXY>& points)
{
    points.clear();
    for (const auto& def : shift_defs) {
        // 该平移点依赖的AR码未识别到，跳过
        if (detected_ids.find(def.ar_id) == detected_ids.end()) {
            ROS_WARN("AR码 %d 未识别到，跳过 %s", def.ar_id, def.name.c_str());
            continue;
        }
        // 获取AR码TF坐标
        std::string ar_frame = AR_TAG_FRAME_PREFIX + std::to_string(def.ar_id);
        tf::StampedTransform transform;
        try {
            listener.waitForTransform(BASE_FRAME, ar_frame, ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform(BASE_FRAME, ar_frame, ros::Time(0), transform);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("获取AR码 %d TF坐标失败: %s", def.ar_id, ex.what());
            continue;
        }
        // 米转毫米，再叠加偏移
        float ar_x = transform.getOrigin().x() * 1000;
        float ar_y = transform.getOrigin().y() * 1000;
        points.push_back({def.name, ar_x + def.x_offset, ar_y});
        ROS_INFO("计算 %s：AR码%d(%.2f, %.2f) + x偏移%.0f → (%.2f, %.2f)",
                 def.name.c_str(), def.ar_id, ar_x, ar_y, def.x_offset,
                 ar_x + def.x_offset, ar_y);
    }
    return !points.empty();
}

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "grab_observe_node");
    ros::NodeHandle nh;

    armmove_client = nh.serviceClient<arm_controller::move>("/goto_position");
    pick_client = nh.serviceClient<std_srvs::Empty>("/swiftpro/on");
    put_client = nh.serviceClient<std_srvs::Empty>("/swiftpro/off");
    // 发出所选 AR 码 TF 坐标的话题（单位 mm，后续抓取节点订阅使用）
    ar_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ar_tag_pose", 1);

    // ===== Step 1: 机械臂移到安全点 =====
    ROS_INFO("===== Step 1: 移至安全点 (%.0f, %.0f, %.0f) =====",
             SAFE_POINT_X, SAFE_POINT_Y, SAFE_POINT_Z);
    if (!arm_move(SAFE_POINT_X, SAFE_POINT_Y, SAFE_POINT_Z)) {
        return -1;
    }

    // ===== Step 2: 机械臂移到观测点 =====
    ROS_INFO("===== Step 2: 移至观测点 (%.0f, %.0f, %.0f) =====",
             OBS_POINT_X, OBS_POINT_Y, OBS_POINT_Z);
    if (!arm_move(OBS_POINT_X, OBS_POINT_Y, OBS_POINT_Z)) {
        return -1;
    }

    // ===== Step 3: 订阅 AR 识别话题 =====
    ros::Subscriber ar_sub = nh.subscribe("hand_camera/ar_pose_marker", 10, arMarkerCallback);

    // ===== Step 4-5: 扫描选择AR码，获取TF坐标，计算平移点（循环直到成功） =====
    int selected_id = -1;
    float ar_x = 0, ar_y = 0;
    std::vector<PointXY> avail_points;
    bool points_ready = false;

    while (ros::ok() && !points_ready) {
        scan_ar_codes();

        // 一个都没识别到：提示重新扫描或退出
        if (detected_ids.empty()) {
            printf("\n未识别到任何AR码，请调整机械臂观测位置后重新扫描。\n");
            printf("  输入 0 退出程序，输入其他任意数字重新扫描：");
            fflush(stdout);
            int c = getChoice();
            if (c == 0) {
                ROS_WARN("程序退出！");
                safe_retract();
                return 0;
            }
            continue;
        }

        // 打印菜单，让用户选择要获取坐标的 AR 码
        printf("\n======================================\n");
        printf("      请选择要获取坐标的AR码\n");
        printf("======================================\n");
        int idx = 1;
        for (int id : detected_ids) {
            printf("  %d - AR码 ID %d\n", idx++, id);
        }
        printf("  0 - 退出程序\n");
        printf("======================================\n");
        printf("请输入编号：");
        fflush(stdout);
        int choice = getChoice();

        if (choice == 0) {
            ROS_WARN("程序退出！");
            safe_retract();
            return 0;
        }
        if (choice < 1 || choice > (int)detected_ids.size()) {
            ROS_ERROR("编号无效，请重新选择！");
            continue;
        }
        auto it = detected_ids.begin();
        std::advance(it, choice - 1);
        selected_id = *it;

        // 获取所选 AR 码的 TF 坐标并发出
        ROS_INFO("===== 获取AR码 ID %d 的TF坐标 =====", selected_id);
        tf::TransformListener listener;
        tf::StampedTransform transform;
        std::string ar_frame = AR_TAG_FRAME_PREFIX + std::to_string(selected_id);

        try {
            listener.waitForTransform(BASE_FRAME, ar_frame, ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform(BASE_FRAME, ar_frame, ros::Time(0), transform);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("获取AR码TF坐标失败: %s", ex.what());
            safe_retract();
            return -1;
        }

        // TF 单位是米，转换为机械臂坐标系下的毫米
        ar_x = transform.getOrigin().x() * 1000;
        ar_y = transform.getOrigin().y() * 1000;
        float ar_z = transform.getOrigin().z() * 1000;

        ROS_INFO("AR码 ID %d 位置: (%.2f, %.2f, %.2f)mm", selected_id, ar_x, ar_y, ar_z);

        // 组装并发布 TF 坐标（含位置与姿态）
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = BASE_FRAME;
        pose.pose.position.x = ar_x;
        pose.pose.position.y = ar_y;
        pose.pose.position.z = ar_z;
        pose.pose.orientation.x = transform.getRotation().x();
        pose.pose.orientation.y = transform.getRotation().y();
        pose.pose.orientation.z = transform.getRotation().z();
        pose.pose.orientation.w = transform.getRotation().w();

        ar_pose_pub.publish(pose);
        ROS_INFO("已发出AR码 ID %d 的TF坐标到话题 /ar_tag_pose", selected_id);

        // 根据 AR码100/101 的 TF 坐标计算平移点
        if (!compute_shift_points(listener, avail_points)) {
            ROS_ERROR("未识别到AR码100或101，无法计算平移点，请重新扫描！");
            continue;
        }
        points_ready = true;
    }
    if (!ros::ok()) {
        return -1;
    }

    // ===== Step 6: 平移到AR码正上方（高度保持观测点高度，防碰撞） =====
    ROS_INFO("===== Step 6: 平移到AR码正上方 (%.2f, %.2f, %.0f) =====",
             ar_x, ar_y, OBS_POINT_Z);
    if (!arm_move(ar_x, ar_y, OBS_POINT_Z)) {
        safe_retract();
        return -1;
    }
    ros::Duration(1.0).sleep();

    // ===== Step 7: 平移/抓取菜单（不退出程序，可反复选择） =====
    bool retract_to_safe = false;
    // 当前机械臂位置（初始为AR码正上方），供抓取功能使用
    float cur_x = ar_x, cur_y = ar_y;
    while (ros::ok()) {
        printf("\n======================================\n");
        printf("      请选择平移目标点\n");
        printf("======================================\n");
        for (int i = 0; i < (int)avail_points.size(); i++) {
            printf("  %d - %s (%.2f, %.2f, %.0f)\n", i + 1,
                   avail_points[i].name.c_str(),
                   avail_points[i].x, avail_points[i].y, OBS_POINT_Z);
        }
        printf("  %d - 开启抓取功能（在当前点下降吸取，再选择放置点放置）\n",
               (int)avail_points.size() + 1);
        printf("  %d - 回到安全点并退出\n", (int)avail_points.size() + 2);
        printf("  0 - 保持当前位置退出\n");
        printf("======================================\n");
        printf("请输入编号：");
        fflush(stdout);

        int c = getChoice();

        if (c == 0) {
            break;   // 保持当前位置退出
        }
        if (c == (int)avail_points.size() + 2) {
            retract_to_safe = true;   // 回到安全点后退出
            break;
        }
        if (c == (int)avail_points.size() + 1) {
            do_grab(cur_x, cur_y);   // 开启抓取功能
            continue;
        }
        if (c < 1 || c > (int)avail_points.size()) {
            ROS_ERROR("输入无效，请重新选择！");
            continue;
        }

        // 平移到所选目标点（高度保持观测点高度，防碰撞）
        PointXY& sp = avail_points[c - 1];
        ROS_INFO("平移至 %s (%.2f, %.2f, %.0f)...", sp.name.c_str(), sp.x, sp.y, OBS_POINT_Z);
        if (!arm_move(sp.x, sp.y, OBS_POINT_Z)) {
            ROS_ERROR("平移失败：%s", sp.name.c_str());
            continue;
        }
        cur_x = sp.x;
        cur_y = sp.y;
        ROS_INFO("已到达 %s，可继续选择其他目标点", sp.name.c_str());
    }

    // ===== Step 8: 退出处理 =====
    if (retract_to_safe) {
        ROS_INFO("===== 回到安全点 (%.0f, %.0f, %.0f) =====",
                 SAFE_POINT_X, SAFE_POINT_Y, SAFE_POINT_Z);
        if (!arm_move(SAFE_POINT_X, SAFE_POINT_Y, SAFE_POINT_Z)) {
            return -1;
        }
        ROS_INFO("===== 任务完成！已回到安全点 =====");
    } else {
        ROS_WARN("程序退出，机械臂保持当前位置！");
    }
    return 0;
}
