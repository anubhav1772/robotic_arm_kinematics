#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometric_shapes/shape_operations.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class ArmMotion
{
public:
    explicit ArmMotion(ros::NodeHandle nh);
    ~ArmMotion();

private:
    ros::NodeHandle nh_;

    const std::string ARM_PLANNING_GROUP = "arm_manipulator";
    const std::string GRIPPER_PLANNING_GROUP = "gripper";

    const std::string DROPBOX_MESH_PATH = "package://kuka_arm_gazebo/models/dropbox/meshes/dropbox.dae";

    std::vector<std::string> OBJECT_MESH_PATH_LIST;
    const std::string OBJECT_MESH_PATH = "package://kuka_arm_gazebo/models/tomato_sauce/meshes/tomato_sauce.dae";

    moveit::planning_interface::MoveGroupInterface arm_move_group;
    moveit::planning_interface::MoveGroupInterface gripper_move_group;

    const robot_state::JointModelGroup *arm_joint_model_group;
    const robot_state::JointModelGroup *gripper_joint_model_group;

    // Define PlanningSceneInterface object to add and remove collision objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;

    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // moveit_visual_tools::MoveItVisualToolsPtr visual_tools_ptr;
    ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Publisher world_joint_pub;

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

    /*
        Functions for gripper actuation
        close_gripper = 0; open gripper
                      = 1; close gripper
    */
    bool OperateGripper(const bool &close_gripper, float value);

    void SetupCollisionObject(const std::string &object_id,
                              const std::string &mesh_path,
                              const geometry_msgs::Pose &object_pose,
                              moveit_msgs::CollisionObject &collision_object);

    tf2::Quaternion RPYToQuaternion(float R, float P, float Y);
};