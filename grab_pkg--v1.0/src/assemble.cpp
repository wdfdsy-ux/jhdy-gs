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

// 装配任务观测点坐标（机械臂坐标系，单位 mm）
const float OBS_POINT_X = 135.0;
const float OBS_POINT_Y = -185.0;
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

// 装配点定义：名称 + 基准AR码ID + x方向偏移
// 装配点一由AR码100换算（x-75），装配点二由AR码101换算（x-75）
// 计算规则：装配点坐标 = (AR码TF的x + x偏移, AR码TF的y)
struct AssembleDef {
    std::string name;
    int ar_id;
    float x_offset;
};

// 装配点定义列表
std::vector<AssembleDef> assemble_defs = {
    {"装配点一", 100, -75},
    {"装配点二", 101, -75},
};

// 计算出的装配点（运行时根据AR码TF坐标填充）
struct PointXY {
    std::string name;
    float x, y;
};

// 中转箱结构体：名称 + 坐标(x,y,z)（坐标与 grab_1.cpp / lay_1.cpp 的放置点一致）
struct BoxPoint {
    std::string name;
    float x, y, z;
};

// 中转箱列表（编号从 1 开始，与终端菜单选项对应）
std::vector<BoxPoint> box_points = {
    {"中转箱1", 107.0, 115.0, 42.0},
    {"中转箱2", 107.0, 185.0, 42.0},
};

// 装配参数
const float ASSEMBLE_DOWN_Z = 40.0;      // 装配点放置高度：从85mm转移高度下降45mm
const float PLACE_LIFT_OFFSET = 30.0;    // 放置后出刀上升距离（防撞）

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

// AR 标签订阅回调：收集视野内所有 AR 码 ID
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

// 取货功能：在中转箱吸取物块
// 在85mm高度平移到中转箱上方 → 下降至中转箱高度(z=42) → 开启吸盘 → 上升回85mm
// 返回是否成功（成功后才进入装配环节）
bool do_pickup(BoxPoint& box, float& cur_x, float& cur_y)
{
    ROS_INFO("===== 取货功能开始：%s (%.1f, %.1f, %.1f) =====",
             box.name.c_str(), box.x, box.y, box.z);

    // 在85mm转移高度平移到中转箱上方
    ROS_INFO("平移到 %s 上方 (%.1f, %.1f, %.0f)", box.name.c_str(), box.x, box.y, OBS_POINT_Z);
    if (!arm_move(box.x, box.y, OBS_POINT_Z)) {
        ROS_ERROR("平移到中转箱失败，取货中止！");
        return false;
    }

    // 下降至中转箱高度，开启吸盘吸取
    ROS_INFO("下降至取货高度 (%.1f, %.1f, %.1f)", box.x, box.y, box.z);
    if (!arm_move(box.x, box.y, box.z)) {
        ROS_ERROR("下降失败，取货中止！");
        return false;
    }
    ros::Duration(1.0).sleep();

    ROS_INFO("开启吸盘，吸取物块");
    set_pump(true);
    ros::Duration(1.0).sleep();

    // 上升至85mm转移高度
    ROS_INFO("上升至转移高度 (%.1f, %.1f, %.0f)", box.x, box.y, OBS_POINT_Z);
    if (!arm_move(box.x, box.y, OBS_POINT_Z)) {
        ROS_ERROR("上升失败，取货中止！");
        return false;
    }
    cur_x = box.x;
    cur_y = box.y;

    ROS_INFO("===== 取货完成！ =====");
    return true;
}

// 装配功能：转移到装配点正上方，下降45mm至z=40放置
void do_place(PointXY& target, float& cur_x, float& cur_y)
{
    ROS_INFO("===== 装配功能开始：%s (%.2f, %.2f) =====",
             target.name.c_str(), target.x, target.y);

    // 在85mm转移高度平移到装配点正上方
    ROS_INFO("平移到 %s 上方 (%.2f, %.2f, %.0f)", target.name.c_str(),
             target.x, target.y, OBS_POINT_Z);
    if (!arm_move(target.x, target.y, OBS_POINT_Z)) {
        ROS_ERROR("平移到装配点失败，装配中止！");
        return;
    }

    // 下降45mm至z=40，放置
    ROS_INFO("下降至装配高度 (%.2f, %.2f, %.0f)", target.x, target.y, ASSEMBLE_DOWN_Z);
    if (!arm_move(target.x, target.y, ASSEMBLE_DOWN_Z)) {
        ROS_ERROR("下降装配失败，装配中止！");
        return;
    }
    ros::Duration(1.0).sleep();

    ROS_INFO("关闭吸盘，放置物块");
    set_pump(false);
    ros::Duration(1.0).sleep();

    // 出刀：上升30mm防撞，并记录当前位置
    float lift_z = ASSEMBLE_DOWN_Z + PLACE_LIFT_OFFSET;
    ROS_INFO("出刀，上升至 (%.2f, %.2f, %.0f)", target.x, target.y, lift_z);
    if (!arm_move(target.x, target.y, lift_z)) {
        ROS_WARN("出刀失败，请手动检查机械臂！");
    }
    cur_x = target.x;
    cur_y = target.y;

    ROS_INFO("===== 装配完成！ =====");
}

// 中转取货功能：在装配点吸取装配好的物件
// 在85mm高度平移到装配点上方 → 下降至装配高度(z=40) → 开启吸盘 → 上升回85mm
// 返回是否成功
bool do_transfer_pickup(PointXY& target, float& cur_x, float& cur_y)
{
    ROS_INFO("===== 中转取货开始：%s (%.2f, %.2f) =====",
             target.name.c_str(), target.x, target.y);

    // 在85mm转移高度平移到装配点上方
    ROS_INFO("平移到 %s 上方 (%.2f, %.2f, %.0f)", target.name.c_str(),
             target.x, target.y, OBS_POINT_Z);
    if (!arm_move(target.x, target.y, OBS_POINT_Z)) {
        ROS_ERROR("平移到装配点失败，中转取货中止！");
        return false;
    }

    // 下降至装配高度，开启吸盘吸取
    ROS_INFO("下降至取货高度 (%.2f, %.2f, %.0f)", target.x, target.y, ASSEMBLE_DOWN_Z);
    if (!arm_move(target.x, target.y, ASSEMBLE_DOWN_Z)) {
        ROS_ERROR("下降失败，中转取货中止！");
        return false;
    }
    ros::Duration(1.0).sleep();

    ROS_INFO("开启吸盘，吸取物块");
    set_pump(true);
    ros::Duration(1.0).sleep();

    // 上升至85mm转移高度
    ROS_INFO("上升至转移高度 (%.2f, %.2f, %.0f)", target.x, target.y, OBS_POINT_Z);
    if (!arm_move(target.x, target.y, OBS_POINT_Z)) {
        ROS_ERROR("上升失败，中转取货中止！");
        return false;
    }
    cur_x = target.x;
    cur_y = target.y;

    ROS_INFO("===== 中转取货完成！ =====");
    return true;
}

// 中转放置功能：在中转箱放置物块
// 在85mm高度平移到中转箱上方 → 下降至中转箱高度(z=42) → 关闭吸盘 → 出刀上升30mm
void do_transfer_place(BoxPoint& box, float& cur_x, float& cur_y)
{
    ROS_INFO("===== 中转放置开始：%s (%.1f, %.1f) =====",
             box.name.c_str(), box.x, box.y);

    // 在85mm转移高度平移到中转箱上方
    ROS_INFO("平移到 %s 上方 (%.1f, %.1f, %.0f)", box.name.c_str(), box.x, box.y, OBS_POINT_Z);
    if (!arm_move(box.x, box.y, OBS_POINT_Z)) {
        ROS_ERROR("平移到中转箱失败，中转放置中止！");
        return;
    }

    // 下降至中转箱高度，放置
    ROS_INFO("下降至放置高度 (%.1f, %.1f, %.1f)", box.x, box.y, box.z);
    if (!arm_move(box.x, box.y, box.z)) {
        ROS_ERROR("下降放置失败，中转放置中止！");
        return;
    }
    ros::Duration(1.0).sleep();

    ROS_INFO("关闭吸盘，放置物块");
    set_pump(false);
    ros::Duration(1.0).sleep();

    // 出刀：上升30mm防撞，并记录当前位置
    float lift_z = box.z + PLACE_LIFT_OFFSET;
    ROS_INFO("出刀，上升至 (%.1f, %.1f, %.0f)", box.x, box.y, lift_z);
    if (!arm_move(box.x, box.y, lift_z)) {
        ROS_WARN("出刀失败，请手动检查机械臂！");
    }
    cur_x = box.x;
    cur_y = box.y;

    ROS_INFO("===== 中转放置完成！ =====");
}

// 根据AR码TF坐标测算装配点：装配点 = (AR码TF的x + x偏移, AR码TF的y)
// 只计算识别到的AR码（100/101）对应的装配点，返回是否至少算出一个点
bool compute_assemble_points(tf::TransformListener& listener, std::vector<PointXY>& points)
{
    points.clear();
    for (const auto& def : assemble_defs) {
        // 该装配点依赖的AR码未识别到，跳过
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
        ROS_INFO("测算 %s：AR码%d(%.2f, %.2f) + x偏移%.0f → (%.2f, %.2f)",
                 def.name.c_str(), def.ar_id, ar_x, ar_y, def.x_offset,
                 ar_x + def.x_offset, ar_y);
    }
    return !points.empty();
}

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "assemble_node");
    ros::NodeHandle nh;

    armmove_client = nh.serviceClient<arm_controller::move>("/goto_position");
    pick_client = nh.serviceClient<std_srvs::Empty>("/swiftpro/on");
    put_client = nh.serviceClient<std_srvs::Empty>("/swiftpro/off");
    // 发出所选 AR 码 TF 坐标的话题（单位 mm，后续装配环节订阅使用）
    ar_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ar_tag_pose", 1);

    // ===== Step 1: 机械臂移到安全点 =====
    ROS_INFO("===== Step 1: 移至安全点 (%.0f, %.0f, %.0f) =====",
             SAFE_POINT_X, SAFE_POINT_Y, SAFE_POINT_Z);
    if (!arm_move(SAFE_POINT_X, SAFE_POINT_Y, SAFE_POINT_Z)) {
        return -1;
    }

    // ===== Step 2: 机械臂移到装配观测点 =====
    ROS_INFO("===== Step 2: 移至装配观测点 (%.0f, %.0f, %.0f) =====",
             OBS_POINT_X, OBS_POINT_Y, OBS_POINT_Z);
    if (!arm_move(OBS_POINT_X, OBS_POINT_Y, OBS_POINT_Z)) {
        return -1;
    }

    // ===== Step 3: 订阅 AR 识别话题 =====
    ros::Subscriber ar_sub = nh.subscribe("hand_camera/ar_pose_marker", 10, arMarkerCallback);

    // ===== Step 4: 扫描 + 菜单选择 AR 码 =====
    int selected_id = -1;
    while (ros::ok()) {
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
        break;
    }

    // ===== Step 5: 获取所选 AR 码的 TF 坐标并发出 =====
    ROS_INFO("===== Step 5: 获取AR码 ID %d 的TF坐标 =====", selected_id);
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
    float ar_x = transform.getOrigin().x() * 1000;
    float ar_y = transform.getOrigin().y() * 1000;
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

    // ===== Step 6: 测算装配点位置（以AR码100/101为基准） =====
    ROS_INFO("===== Step 6: 测算装配点位置 =====");
    std::vector<PointXY> avail_points;
    if (!compute_assemble_points(listener, avail_points)) {
        ROS_WARN("未识别到AR码100/101，无法测算装配点！");
    }

    // 打印装配点测算结果
    if (!avail_points.empty()) {
        printf("\n========== 装配点测算结果 ==========\n");
        for (const auto& p : avail_points) {
            printf("  %s: (%.2f, %.2f)mm\n", p.name.c_str(), p.x, p.y);
        }
        printf("====================================\n");
    } else {
        ROS_ERROR("无可用的装配点，程序退出！");
        safe_retract();
        return -1;
    }

    // ===== Step 7: 功能选择（退出 / 中转 / 装配） =====
    int mode = -1;
    while (mode != 0 && mode != 1 && mode != 2) {
        printf("\n======================================\n");
        printf("      请选择功能\n");
        printf("======================================\n");
        printf("  1 - 中转（装配点 → 中转箱）\n");
        printf("  2 - 装配（中转箱 → 装配点）\n");
        printf("  0 - 退出程序\n");
        printf("======================================\n");
        printf("请输入编号：");
        fflush(stdout);
        mode = getChoice();
        if (mode != 0 && mode != 1 && mode != 2) {
            ROS_ERROR("输入无效，请重新选择！");
        }
    }
    if (mode == 0) {
        ROS_WARN("程序退出！");
        safe_retract();
        return 0;
    }

    // ===== Step 8: 执行所选功能（不退出程序，可反复执行） =====
    bool retract_to_safe = false;
    bool running = true;
    float cur_x = ar_x, cur_y = ar_y;

    while (ros::ok() && running) {
        if (mode == 1) {
            // ===== 中转模式：装配点 → 中转箱 =====
            // ----- 取货菜单：选择装配点 -----
            bool picked = false;
            while (ros::ok() && !picked) {
                printf("\n======================================\n");
                printf("      请选择取货的装配点\n");
                printf("======================================\n");
                for (int i = 0; i < (int)avail_points.size(); i++) {
                    printf("  %d - %s (%.2f, %.2f, %.0f)\n", i + 1,
                           avail_points[i].name.c_str(),
                           avail_points[i].x, avail_points[i].y, OBS_POINT_Z);
                }
                printf("  %d - 回到安全点并退出\n", (int)avail_points.size() + 1);
                printf("  0 - 保持当前位置退出\n");
                printf("======================================\n");
                printf("请输入编号：");
                fflush(stdout);

                int c = getChoice();
                if (c == 0) {
                    running = false;   // 保持当前位置退出
                    break;
                }
                if (c == (int)avail_points.size() + 1) {
                    retract_to_safe = true;   // 回到安全点后退出
                    running = false;
                    break;
                }
                if (c < 1 || c > (int)avail_points.size()) {
                    ROS_ERROR("输入无效，请重新选择！");
                    continue;
                }
                picked = do_transfer_pickup(avail_points[c - 1], cur_x, cur_y);
                // 取货失败则留在取货菜单重新选择
            }
            if (!running) {
                break;
            }

            // ----- 放置菜单：选择中转箱 -----
            bool placed = false;
            while (ros::ok() && !placed) {
                printf("\n======================================\n");
                printf("      请选择放置的中转箱\n");
                printf("======================================\n");
                for (int i = 0; i < (int)box_points.size(); i++) {
                    printf("  %d - %s (%.1f, %.1f, %.1f)\n", i + 1,
                           box_points[i].name.c_str(),
                           box_points[i].x, box_points[i].y, box_points[i].z);
                }
                printf("  %d - 回到安全点并退出\n", (int)box_points.size() + 1);
                printf("  0 - 保持当前位置退出\n");
                printf("======================================\n");
                printf("请输入编号：");
                fflush(stdout);

                int c = getChoice();
                if (c == 0) {
                    running = false;   // 保持当前位置退出
                    break;
                }
                if (c == (int)box_points.size() + 1) {
                    retract_to_safe = true;   // 回到安全点后退出
                    running = false;
                    break;
                }
                if (c < 1 || c > (int)box_points.size()) {
                    ROS_ERROR("输入无效，请重新选择！");
                    continue;
                }
                do_transfer_place(box_points[c - 1], cur_x, cur_y);
                placed = true;   // 放置完成后回到取货菜单，可继续搬运
            }
        } else {
            // ===== 装配模式：中转箱 → 装配点（原有逻辑） =====
            // ----- 取货菜单：选择中转箱 -----
            bool picked = false;
            while (ros::ok() && !picked) {
                printf("\n======================================\n");
                printf("      请选择取货的中转箱\n");
                printf("======================================\n");
                for (int i = 0; i < (int)box_points.size(); i++) {
                    printf("  %d - %s (%.1f, %.1f, %.1f)\n", i + 1,
                           box_points[i].name.c_str(),
                           box_points[i].x, box_points[i].y, box_points[i].z);
                }
                printf("  %d - 回到安全点并退出\n", (int)box_points.size() + 1);
                printf("  0 - 保持当前位置退出\n");
                printf("======================================\n");
                printf("请输入编号：");
                fflush(stdout);

                int c = getChoice();
                if (c == 0) {
                    running = false;   // 保持当前位置退出
                    break;
                }
                if (c == (int)box_points.size() + 1) {
                    retract_to_safe = true;   // 回到安全点后退出
                    running = false;
                    break;
                }
                if (c < 1 || c > (int)box_points.size()) {
                    ROS_ERROR("输入无效，请重新选择！");
                    continue;
                }
                picked = do_pickup(box_points[c - 1], cur_x, cur_y);
                // 取货失败则留在取货菜单重新选择
            }
            if (!running) {
                break;
            }

            // ----- 装配菜单：选择装配点 -----
            bool placed = false;
            while (ros::ok() && !placed) {
                printf("\n======================================\n");
                printf("      请选择装配点\n");
                printf("======================================\n");
                for (int i = 0; i < (int)avail_points.size(); i++) {
                    printf("  %d - %s (%.2f, %.2f, %.0f)\n", i + 1,
                           avail_points[i].name.c_str(),
                           avail_points[i].x, avail_points[i].y, OBS_POINT_Z);
                }
                printf("  %d - 回到安全点并退出\n", (int)avail_points.size() + 1);
                printf("  0 - 保持当前位置退出\n");
                printf("======================================\n");
                printf("请输入编号：");
                fflush(stdout);

                int c = getChoice();
                if (c == 0) {
                    running = false;   // 保持当前位置退出
                    break;
                }
                if (c == (int)avail_points.size() + 1) {
                    retract_to_safe = true;   // 回到安全点后退出
                    running = false;
                    break;
                }
                if (c < 1 || c > (int)avail_points.size()) {
                    ROS_ERROR("输入无效，请重新选择！");
                    continue;
                }
                do_place(avail_points[c - 1], cur_x, cur_y);
                placed = true;   // 装配完成后回到取货菜单，可继续搬运
            }
        }
    }

    // ===== 退出处理 =====
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
