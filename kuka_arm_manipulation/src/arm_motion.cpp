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

    // slow down movement of the arm
    arm_move_group.setMaxVelocityScalingFactor(0.2);
    gripper_move_group.setMaxVelocityScalingFactor(1.0);

    // 5 cm (0.05m)
    arm_move_group.setGoalTolerance(0.05);
    
    bool success = static_cast<bool>(arm_move_group.plan(arm_plan));
    ROS_INFO("Plan to target: %s", success ? "SUCCEEDED" : "FAILED");

    arm_move_group.execute(arm_plan);
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
