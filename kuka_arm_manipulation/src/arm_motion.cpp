#include "kuka_arm_manipulation/arm_motion.h"

ArmMotion::ArmMotion(ros::NodeHandle nh)
    : nh_(nh),
      arm_move_group(ARM_PLANNING_GROUP),
      gripper_move_group(GRIPPER_PLANNING_GROUP)
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    //ROS_INFO_NAMED("ARM MOTION", "Reference frame: %s", arm_move_group.getPlanningFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    //kinematic_state->setToDefaultValues();

    arm_move_group.setPlannerId("RRTkConfigDefault");
    arm_move_group.setPlanningTime(10.0);
    gripper_move_group.setPlannerId("RRTkConfigDefault");
    gripper_move_group.setPlanningTime(5.0);

    arm_joint_model_group = arm_move_group.getCurrentState()->getJointModelGroup("arm_manipulator");
    gripper_joint_model_group = kinematic_model->getJointModelGroup("gripper");

    




  /*
    // arm_move_group.setPlanningFrame("base_link");
    // Pointer to JointModelGroup for improved performance.
    arm_joint_model_group = arm_move_group.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);
    gripper_joint_model_group = gripper_move_group.getCurrentState()->getJointModelGroup(GRIPPER_PLANNING_GROUP);

    //ROS_INFO_STREAM(gripper_move_group.getCurrentState());

    // We can print the name of the reference frame for this robot.
    //ROS_INFO_NAMED("ARM MOTION", "Reference frame: %s", arm_move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    //ROS_INFO_NAMED("ARM MOTION", "End effector link: %s", arm_move_group.getEndEffectorLink().c_str());

    robot_model_loader = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader->getModel();

    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    
    
    
    planning_scene_monitor = planning_scene_monitor::PlanningSceneMonitorPtr(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    */
    /*
    std::vector<geometry_msgs::Pose> mesh_pose_list;
    geometry_msgs::Pose target_mesh_pose;

    // Mesh poses
    // tomato sauce can 
    OBJECT_MESH_PATH_LIST.push_back(OBJECT_MESH_PATH);

    // cracker pose
    // 0.854077 -0.02371 1.234 -0.040365 -0.000621 -1.5396
    tf2::Quaternion qt = RPYToQuaternion(-0.040365, -0.000621, -1.539607);
    target_mesh_pose.position.x = 0.854077;
    target_mesh_pose.position.y = -0.02371;
    target_mesh_pose.position.z = 1.234;
    target_mesh_pose.orientation.w = qt.getW();
    target_mesh_pose.orientation.x = qt.getX();
    target_mesh_pose.orientation.y = qt.getY();
    target_mesh_pose.orientation.z = qt.getZ();
    mesh_pose_list.push_back(target_mesh_pose);

    // Add object to the scene
    std::vector<moveit_msgs::CollisionObject> target_object_list;
    for (size_t i = 0; i < 1; ++i)
    {
        moveit_msgs::CollisionObject target_collision_object;
        std::string target_id;
        target_id = "target" + std::to_string(i);
        SetupCollisionObject(target_id, OBJECT_MESH_PATH_LIST[i], mesh_pose_list[i], target_collision_object);
        target_object_list.push_back(target_collision_object);
    }

    planning_scene_interface.applyCollisionObjects(collision_objects);
    ros::Duration(0.1).sleep();

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    geometry_msgs::Pose target_pose;

    target_pose.orientation.w = 0.4947743972838332;
    target_pose.orientation.x = -0.4952133362195981;
    target_pose.orientation.y = 0.5001244266339454;
    target_pose.orientation.z = -0.5097426853881694;
    target_pose.position.x = 0.61245;
    target_pose.position.y = -0.01091;
    target_pose.position.z = 1.43022;

    // set starting pose
    arm_move_group.setStartStateToCurrentState();
    // set target pose
    arm_move_group.setPoseTarget(target_pose);
    // 5 cm (0.05m)
    arm_move_group.setGoalTolerance(0.05);
    bool success = static_cast<bool>(arm_move_group.plan(arm_plan));
    ROS_INFO("Plan to target 1: %s", success ? "SUCCEEDED" : "FAILED");

    arm_move_group.execute(arm_plan);

    target_pose.orientation.w = 0.4947874914183782;
    target_pose.orientation.x = -0.49524148204092966;
    target_pose.orientation.y = 0.5001619108730555;
    target_pose.orientation.z = -0.5096658471132098;
    target_pose.position.x = 0.61342;
    target_pose.position.y = -0.01187;
    target_pose.position.z = 1.33021;

    arm_move_group.setStartStateToCurrentState();
    arm_move_group.setGoalTolerance(0.05);
    arm_move_group.setPoseTarget(target_pose);
    success = static_cast<bool>(arm_move_group.plan(arm_plan));
    ROS_INFO("Plan to target 2: %s", success ? "SUCCEEDED" : "FAILED");

    arm_move_group.execute(arm_plan);

    // close_gripper == false means open gripper action
    // value can decrease from 0.06 -> 0.0
    // OperateGripper(false, 0.4);   

    // close_gripper == true means close gripper action
    // value can increase from 0.0 -> 0.06
    OperateGripper(true, 0.04);

    target_pose.orientation.w = 0.6886415929006499;
    target_pose.orientation.x = -0.6942511017470506;
    target_pose.orientation.y = 0.14397427108592917;
    target_pose.orientation.z = -0.1518537899281326;
    target_pose.position.x = 0.23252;
    target_pose.position.y = 0.20698;
    target_pose.position.z = 1.41375;
  
    arm_move_group.setStartStateToCurrentState();
    arm_move_group.setGoalTolerance(0.05);
    arm_move_group.setPoseTarget(target_pose);
    success = static_cast<bool>(arm_move_group.plan(arm_plan));
    ROS_INFO("Plan to target 3 (place object): %s", success ? "SUCCEEDED" : "FAILED");
    */
}

ArmMotion::~ArmMotion() {}

bool ArmMotion::OperateGripper(const bool &close_gripper, float value)
{
    // RobotState contains the current position/velocity/acceleration data
    moveit::core::RobotStatePtr gripper_current_state = gripper_move_group.getCurrentState();

    // Next get the current set of joint values for the group.
    std::vector<double> gripper_joint_positions;
    gripper_current_state->copyJointGroupPositions(gripper_joint_model_group, gripper_joint_positions);

    //ROS_INFO("No. of  gripper joint is %ld",gripper_joint_positions.size());

    gripper_joint_positions[0] = value;
    gripper_move_group.setJointValueTarget(gripper_joint_positions);
    //ros::Duration(1.0).sleep();
    
    bool success;
    if (close_gripper)
    { 
        // close_gripper != false or close_gripper == true
        // gripper_joint_positions[0] = 0.06;
        ROS_INFO_STREAM("Initiating close gripper action...");
        success = static_cast<bool>(gripper_move_group.move());
        ROS_INFO("Close gripper action is %s",(success==true)? "success":"failure");
    }
    else
    {
        // close_gripper == false or open_gripper == true
        // gripper_joint_positions[0] = 0.0;
        ROS_INFO_STREAM("Initiating open gripper action...");
        success = static_cast<bool>(gripper_move_group.move());
        ROS_INFO("Open gripper action is %s",(success==true)? "success":"failure");
    }  
  return success;
}

void ArmMotion::SetupCollisionObject(const std::string &object_id,
                                      const std::string &mesh_path,
                                      const geometry_msgs::Pose &object_pose,
                                      moveit_msgs::CollisionObject &collision_object)
{

    collision_object.header.frame_id = arm_move_group.getPlanningFrame();
    collision_object.id = object_id;

    shapes::Mesh *m = shapes::createMeshFromResource(mesh_path);

    ROS_DEBUG_STREAM(object_id << " mesh loaded");

    shape_msgs::Mesh object_mesh;
    shapes::ShapeMsg object_mesh_msg;
    shapes::constructMsgFromShape(m, object_mesh_msg);
    object_mesh = boost::get<shape_msgs::Mesh>(object_mesh_msg);
    collision_object.meshes.resize(1);
    collision_object.mesh_poses.resize(1);
    collision_object.meshes[0] = object_mesh;

    collision_object.mesh_poses[0].position = object_pose.position;
    collision_object.mesh_poses[0].orientation = object_pose.orientation;

    collision_object.meshes.push_back(object_mesh);
    collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);
}

tf2::Quaternion ArmMotion::RPYToQuaternion(float R, float P, float Y)
{
    tf2::Matrix3x3 mat;
    mat.setEulerYPR(Y, P, R);

    tf2::Quaternion quat;
    mat.getRotation(quat);

    return quat;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_traj_sampler");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ArmMotion arm_motion(nh);

    //ros::waitForShutdown();
    ros::shutdown();
    return 0;
}
