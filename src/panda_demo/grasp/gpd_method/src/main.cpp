// === gpd_method / panda_grasp_executor.cpp (修改版) ===
// 以第一个文件为主，加入第二个文件的夹爪动作方式

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <gpd_ros/GraspConfigList.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>
#include <iostream>

// ★ 新增：夹爪动作客户端
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>

using namespace std;

class GraspExecutor
{
public:
  GraspExecutor()
      : move_group("panda_manipulator"),
        tf_listener(tf_buffer)
  {
    // 发布最终选取的抓取姿态
    best_grasp_pub = nh.advertise<geometry_msgs::PoseStamped>("selected_grasp", 1);
    cout << "19" << endl;

    // 订阅 GPD 输出
    grasps_sub = nh.subscribe<gpd_ros::GraspConfigList>(
        "/detect_grasps/clustered_grasps", 1,
        boost::bind(&GraspExecutor::graspsCallback, this, _1));
    cout << "26" << endl;
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber grasps_sub;
  ros::Publisher best_grasp_pub;
  moveit::planning_interface::MoveGroupInterface move_group;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  void graspsCallback(const gpd_ros::GraspConfigList::ConstPtr &msg)
  {
    if (msg->grasps.empty())
    {
      ROS_WARN("No grasps received!");
      cout << "38" << endl;
      return;
    }

    // 1. 选择分数最高的抓取
    int best_idx = 0;
    double max_score = -1.0;
    cout << "46" << endl;
    for (size_t i = 0; i < msg->grasps.size(); ++i)
    {
      if (msg->grasps[i].score.data > max_score)
      {
        max_score = msg->grasps[i].score.data;
        best_idx = i;
      }
    }
    cout << "53" << endl;
    const auto &g = msg->grasps[best_idx];

    // 2. 构造抓取位姿（相机坐标系）
    geometry_msgs::PoseStamped grasp_in_cam;
    grasp_in_cam.header = msg->header; // 维持真实坐标系
    grasp_in_cam.pose.position.x = g.position.x;
    grasp_in_cam.pose.position.y = g.position.y;
    grasp_in_cam.pose.position.z = g.position.z;
    cout << "63" << endl;
    ROS_INFO("grasp_in_cam.pose: x=%.3f, y=%.3f, z=%.3f",
             g.position.x, g.position.y, g.position.z);

    // —— 用 GPD 的向量构造手系 R：X=binormal, Y=axis, Z=approach —— //
    Eigen::Vector3d a(g.approach.x, g.approach.y, g.approach.z); // Z_hand
    Eigen::Vector3d y(g.axis.x, g.axis.y, g.axis.z);             // Y_hand
    a.normalize();
    y = (y - y.dot(a) * a).normalized();
    Eigen::Vector3d x = y.cross(a).normalized(); // X_hand = Y × Z

    Eigen::Matrix3d R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = a;

    // 在手系内补一个绕 Z_hand 的偏航（缺省 90°），保持你现网格的对齐方式
    double yaw_fix_deg = 90.0;
    double yaw_fix = yaw_fix_deg * M_PI / 180.0;
    Eigen::Matrix3d Rz_hand = Eigen::AngleAxisd(yaw_fix, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    R = R * Rz_hand;

    Eigen::Quaterniond q(R);
    q.normalize();
    grasp_in_cam.pose.orientation.x = q.x();
    grasp_in_cam.pose.orientation.y = q.y();
    grasp_in_cam.pose.orientation.z = q.z();
    grasp_in_cam.pose.orientation.w = q.w();
    cout << "80" << endl;

    // 3. 转换到机器人基座坐标系 (panda_link0)
    geometry_msgs::PoseStamped grasp_in_base;
    try
    {
      tf_buffer.transform(grasp_in_cam, grasp_in_base, "panda_link0");
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("Transform failed: %s", ex.what());
      return;
    }
    cout << "89" << endl;
    ROS_INFO("grasp_in_base.pose: x=%.3f, y=%.3f, z=%.3f",
             grasp_in_base.pose.position.x,
             grasp_in_base.pose.position.y,
             grasp_in_base.pose.position.z);
    ROS_INFO("grasp_in_base.pose: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
             grasp_in_base.pose.orientation.x,
             grasp_in_base.pose.orientation.y,
             grasp_in_base.pose.orientation.z,
             grasp_in_base.pose.orientation.w);
    cout << "100" << endl;
    ROS_INFO("Selected grasp with score: %.2f", max_score);

    // —— 按你原逻辑：抬到正上方 0.3m —— //
    grasp_in_base.pose.position.z += 0.3;
    best_grasp_pub.publish(grasp_in_base);

    std::string reference_frame = "panda_link0";
    move_group.setPoseReferenceFrame(reference_frame);
    cout << "107" << endl;
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0); 
    move_group.setPoseTarget(grasp_in_base);
    moveit::planning_interface::MoveGroupInterface::Plan plan;  

    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      move_group.execute(plan);
    }
    else
    {
      ROS_WARN("Planning failed for selected grasp!");
      cout << "115" << endl;
      return;
    }
    cout << "115" << endl;

    // ====== 从“正上方 0.4m”开始，执行你的抓-放序列 ======

    // A) 夹爪动作客户端（来自第二个文件的方法）
    actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client("/franka_gripper/move", true);
    actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client("/franka_gripper/grasp", true);
    ROS_INFO("Waiting for gripper action servers...");
    if (!move_client.waitForServer(ros::Duration(10.0)) ||
        !grasp_client.waitForServer(ros::Duration(10.0)))
    {
      ROS_ERROR("Gripper action servers not available, abort.");
      return;
    }
    ROS_INFO("Gripper action servers connected.");

    // B) 张开夹爪（释放到最大开度）
    franka_gripper::MoveGoal open_goal;
    open_goal.width = 0.08; // 依据你的真实爪宽配置可调
    open_goal.speed = 1.0;
    move_client.sendGoal(open_goal);
    move_client.waitForResult(ros::Duration(5.0));
    ROS_INFO("Step-OPEN: gripper opened.");

    // C) 向下 0.3 m
    grasp_in_base.pose.position.z -= 0.42;
    geometry_msgs::Pose down = grasp_in_base.pose; // 当前就是“上方 0.3m” 
    move_group.setPoseTarget(down);
    moveit::planning_interface::MoveGroupInterface::Plan plan_down;
    if (move_group.plan(plan_down) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      auto res = move_group.execute(plan_down);
      ROS_INFO("Step-DOWN: executed, code=%d", res.val);
    }
    else
    {
      ROS_ERROR("Step-DOWN: planning failed, abort.");
      return;
    }

    // D) 闭合夹爪（抓取）
    franka_gripper::GraspGoal grasp_goal;
    grasp_goal.width = 0.02;         // 目标抓取宽度（示例）
    grasp_goal.epsilon.inner = 0.01; // 内容差
    grasp_goal.epsilon.outer = 0.01; // 外容差
    grasp_goal.speed = 1.0;          // 闭合速度
    grasp_goal.force = 20.0;         // 抓取力（仿真可放大/缩小）
    grasp_client.sendGoal(grasp_goal);
    ROS_INFO("Step-GRASP: grasp command sent.");

    // E) 向上 0.3 m
    geometry_msgs::Pose up = down;
    up.position.z += 0.3;
    move_group.setPoseTarget(up);
    // 修正：使用正确别名
    // E) 向上 0.3 m（续）
    moveit::planning_interface::MoveGroupInterface::Plan plan_up;
    if (move_group.plan(plan_up) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      auto res = move_group.execute(plan_up);
      ROS_INFO("Step-UP: executed, code=%d", res.val);
    }
    else
    {
      ROS_ERROR("Step-UP: planning failed, abort.");
      return;
    }

    // F) 再次张开夹爪（释放香蕉回落）
    franka_gripper::MoveGoal release_goal;
    release_goal.width = 0.08;
    release_goal.speed = 1.0;
    move_client.sendGoal(release_goal);
    move_client.waitForResult(ros::Duration(5.0));
    ROS_INFO("Step-RELEASE: gripper opened.");

    // G) 结束程序
    ROS_INFO("Sequence done. Shutting down node.");
    ros::shutdown();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "panda_grasp_executor");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ROS_INFO("panda_grasp_executor started.");
  GraspExecutor executor;
  ros::waitForShutdown();
  cout << "126" << endl;
  return 0;
}
