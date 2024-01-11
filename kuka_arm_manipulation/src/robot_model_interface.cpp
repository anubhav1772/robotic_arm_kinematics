/**

  https://github.com/ros-planning/moveit_tutorials/blob/master/doc/robot_model_and_robot_state/src/robot_model_and_robot_state_tutorial.cpp

**/

#include <ros/ros.h>
#include <math.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_interface_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // We will start by instantiating a `RobotModelLoader` object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.
  // RobotModelLoader:
  // http://docs.ros.org/kinetic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::planning_interface::MoveGroupInterface arm_move_group("arm_manipulator");
  ROS_INFO_NAMED("ARM MOTION", "Reference frame: %s", arm_move_group.getPlanningFrame().c_str());

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :moveit_core:`RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  // :moveit_core:`JointModelGroup`, which represents the robot
  // model for a particular group, e.g. the "arm_manipulator" of the owr robot.
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm_manipulator");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // input joint values and receive cartesian position as output
  double current_pos[] = {M_PI/2, M_PI/3, M_PI/2, M_PI/3, M_PI/3, M_PI/2};
  ROS_INFO("Joint pos = [ %f, %f, %f, %f, %f, %f ]", current_pos[0], current_pos[1], current_pos[2], current_pos[3], current_pos[4], current_pos[5] );
  kinematic_state->setVariablePositions(current_pos);
  kinematic_state->update();

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the owr arm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Joint Limits
  // ^^^^^^^^^^^^
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  /* Set one joint in the owr arm outside its joint limit */
  joint_values[0] = 5.57;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  /* Check whether any joint is outside its joint limits */
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  // Forward Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // Now, we can compute forward kinematics for a set of random joint
  // values. Note that we would like to find the pose of the
  // "EEF_Link" which is the most distal link in the
  // "arm_manipulator" group of the robot.
  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state  = kinematic_state->getGlobalLinkTransform("gripper_link");

  /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  // Inverse Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // We can now solve inverse kinematics (IK) for the owr robot.
  // To solve IK, we will need the following:
  //
  //  * The desired pose of the end-effector (by default, this is the last link in the "arm_manipulator" chain):
  //    end_effector_state that we computed in the step above.
  //  * The timeout for each attempt: 0.1 s

  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }
  
  // Get the Jacobian
  // ^^^^^^^^^^^^^^^^
  // We can also get the Jacobian from the :moveit_core:`RobotState`.
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
  // END
  

  ros::shutdown();
  return 0;
}