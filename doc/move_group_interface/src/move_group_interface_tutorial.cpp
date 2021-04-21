/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <chrono>


// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;


void load_bari_in_scene(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, const Eigen::Vector3d center){

  //std::string modelpath = "/home/pc-m/Documents/My-cao/barillet.obj";
  std::string modelpath = "package://geometric_shapes/test/resources/barillet.obj";
  ROS_INFO("mesh loades : %s ",modelpath.c_str());

  static const Eigen::Vector3d scale(0.001, 0.001, 0.001);
  shapes::Mesh* cao_bary = shapes::createMeshFromResource(modelpath,scale);

  ROS_INFO("mesh loades : %i triangles : %s ",cao_bary->triangle_count,modelpath.c_str());


  shape_msgs::Mesh mesh_bary;
  shapes::ShapeMsg mesh_bary_msg;

  shapes::constructMsgFromShape(cao_bary,mesh_bary_msg);
  mesh_bary = boost::get<shape_msgs::Mesh>(mesh_bary_msg);

  int N_bari_x = 2;
  int N_bari_y = 4;
  int N_bari_z = 1;
  for (int xi = 0; xi< N_bari_x;xi++){
    for (int yi = 0; yi< N_bari_y;yi++){
      for (int zi = 0; zi< N_bari_z;zi++){

        // Now let's define a collision object ROS message for the robot to avoid.
        moveit_msgs::CollisionObject collision_object_bari;
        collision_object_bari.header.frame_id = move_group_interface.getPlanningFrame();

        char id_bari[20];
        sprintf(id_bari,"bari%d",xi+N_bari_x*yi);

        // The id of the object is used to identify it.
        collision_object_bari.id = id_bari;

        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        //double res_x = center.x -0.5 + 0.1*xi;
        pose.position.x = 0.5+ 0.04*xi;
        pose.position.y = -0.05+ 0.05*yi;
        pose.position.z = 0.15;

        collision_object_bari.meshes.push_back(mesh_bary);
        collision_object_bari.mesh_poses.push_back(pose);
        collision_object_bari.operation = collision_object_bari.ADD;

        collision_object_baris.push_back(collision_object_bari);
      }
    }
  }
  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_object_baris);


}

void load_carton_in_scene(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, const Eigen::Vector3d center){

  //std::string modelpath = "/home/pc-m/Documents/My-cao/barillet.obj";
  std::string modelpath = "package://geometric_shapes/test/resources/Contenant_barillet.stl";
  ROS_INFO("mesh loades : %s ",modelpath.c_str());

  static const Eigen::Vector3d scale(0.001, 0.001, 0.001);
  shapes::Mesh* cao_bary = shapes::createMeshFromResource(modelpath,scale);

  ROS_INFO("mesh loades : %i triangles : %s ",cao_bary->triangle_count,modelpath.c_str());


  shape_msgs::Mesh mesh_bary;
  shapes::ShapeMsg mesh_bary_msg;

  shapes::constructMsgFromShape(cao_bary,mesh_bary_msg);
  mesh_bary = boost::get<shape_msgs::Mesh>(mesh_bary_msg);

  // Now let's define a collision object ROS message for the robot to avoid.
  moveit_msgs::CollisionObject collision_object_bari;
  collision_object_bari.header.frame_id = move_group_interface.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object_bari.id = "carton";

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose pose;
  pose.orientation.x = 0.5;
  pose.orientation.z = 0.5;
  pose.orientation.w = 0.5;
  pose.orientation.y = 0.5;
  pose.position.x = 0.5;
  pose.position.y = 0.0;
  pose.position.z = 0.3;

  collision_object_bari.meshes.push_back(mesh_bary);
  collision_object_bari.mesh_poses.push_back(pose);
  collision_object_bari.operation = collision_object_bari.ADD;

  collision_object_baris.push_back(collision_object_bari);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_object_baris);


}


std::vector<double> go_to_position(moveit::planning_interface::MoveGroupInterface &move_group_interface ,geometry_msgs::Pose target_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group){
  

    // - AnytimePathShortening
    // - SBL
    // - EST
    // - LBKPIECE
    // - BKPIECE
    // - KPIECE
    // - RRT
    // - RRTConnect
    // - RRTstar
    // - TRRT
    // - PRM
    // - PRMstar
    // - FMT
    // - BFMT
    // - PDST
    // - STRIDE
    // - BiTRRT
    // - LBTRRT
    // - BiEST
    // - ProjEST
    // - LazyPRM
    // - LazyPRMstar
    // - SPARS
    // - SPARStwo
    // - PersistentLazyPRMstar
    // - PersistentLazyPRM
    // - SemiPersistentLazyPRMstar
    // - SemiPersistentLazyPRM



  std::string planner_id = "RRTconnect";
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setPlanningTime(60);
  /** \brief Set a scaling factor for optionally reducing the maximum joint velocity.
      Allowed values are in (0,1]. The maximum joint velocity specified
      in the robot model is multiplied by the factor. If the value is 0, it is set to
      the default value, which is defined in joint_limits.yaml of the moveit_config.
      If the value is greater than 1, it is set to 1.0. */
  move_group_interface.setMaxVelocityScalingFactor(1.0);

  /** \brief Set a scaling factor for optionally reducing the maximum joint acceleration.
      Allowed values are in (0,1]. The maximum joint acceleration specified
      in the robot model is multiplied by the factor. If the value is 0, it is set to
      the default value, which is defined in joint_limits.yaml of the moveit_config.
      If the value is greater than 1, it is set to 1.0. */
  move_group_interface.setMaxAccelerationScalingFactor(1.0);


  std::string planner_test = move_group_interface.getPlannerId();
  std::cout<<"getPlannerId : "<<planner_test<<std::endl;

  //std::cout<<planner_test*<<std::endl;


  move_group_interface.setPoseTarget(target_pose);
  // /** \brief Get the current joint state goal in a form compatible to setJointValueTarget() */
  // void getJointValueTarget(std::vector<double>& group_variable_values) const;


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


  ROS_INFO_NAMED("tutorial", " Go to scan position %s", success ? "" : "FAILED");

  std::cout<< "TIME TO FIND THE PATH : " << my_plan.planning_time_ <<std::endl;

  std::chrono::high_resolution_clock::time_point begin_traj = std::chrono::high_resolution_clock::now();
  move_group_interface.execute(my_plan);
  std::chrono::high_resolution_clock::time_point end_traj = std::chrono::high_resolution_clock::now();

  std::cout<< "TIME TO EXECUTE TRAJ : " << std::chrono::duration_cast<std::chrono::microseconds>(end_traj-begin_traj).count()/1000000.0 <<std::endl;





  robot_trajectory::RobotTrajectory trajecto_state(move_group_interface.getCurrentState()->getRobotModel(), move_group_interface.getName());

  trajecto_state.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), my_plan.trajectory_);

  //ROS_INFO_NAMED("trajectory print : ", " test size : %d ", trajecto_state.getWayPointCount() );

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  trajecto_state.getWayPointPtr(0)->copyJointGroupPositions(joint_model_group, joint_group_positions);
  int number_joint = 7;

  for (int i =0 ; i< number_joint  ; i++ )
  {
    std::cout << "angle joint i = " << i << " : " << joint_group_positions[i]*180/3.14159265 << std::endl;
  }
  //std::cout << "test std::cout" << trajecto_state.getWayPoint(5) << std::endl;  //trajecto_state.getWayPointCount()


  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);


  return joint_group_positions;
}




int main(int argc, char** argv)
{








  
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "arm_group";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);











  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Demo MoveIt", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));






  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // Adding objects to the environment
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // First let's plan to another simple goal with no objects in the way.

  geometry_msgs::Pose scan_pose;
  scan_pose.orientation.x = 1.0;
  scan_pose.position.x = 0.75;
  scan_pose.position.y = 0.0;
  scan_pose.position.z = 0.5;


  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, " current : show scan position", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishAxisLabeled(scan_pose, "scan_pose");
  visual_tools.trigger(); // to apply changes





  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



  move_group_interface.setStartState(*move_group_interface.getCurrentState());


  std::vector<double> traj1 = go_to_position(move_group_interface, scan_pose, visual_tools, joint_model_group);

  visual_tools.publishText(text_pose, "Go to scan position", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  const Eigen::Vector3d center(0.5,0.0,0.0);
  //center.x;

  std::vector<moveit_msgs::CollisionObject> collision_object_baris;

  load_bari_in_scene(planning_scene_interface, move_group_interface, collision_object_baris, center);
  load_carton_in_scene(planning_scene_interface, move_group_interface, collision_object_baris, center);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  //ROS_INFO_NAMED("tutorial", "Add an object into the world");
  //planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();



  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");


  int N_bari_x = 2;
  int N_bari_y = 4;
  int N_bari_z = 1;
  for (int xi = 0; xi< N_bari_x;xi++){
    for (int yi = 0; yi< N_bari_y;yi++){
      for (int zi = 0; zi< N_bari_z;zi++){



      geometry_msgs::Pose bari_pose;
      bari_pose.orientation.x = 0.93373;
      bari_pose.orientation.y = -0.35765;
      bari_pose.orientation.z = 0.0057657;
      bari_pose.orientation.w = 0.014457;



      bari_pose.position.x = 0.5+ 0.04*xi;
      bari_pose.position.y = -0.05+ 0.05*yi;
      bari_pose.position.z = 0.53731+0.3;



      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, " current : show bari position", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishAxisLabeled(bari_pose, "bari_pose");
      visual_tools.trigger(); // to apply changes





      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");






      move_group_interface.setStartState(*move_group_interface.getCurrentState());




      std::vector<double> traj2 = go_to_position(move_group_interface, bari_pose, visual_tools, joint_model_group);

      visual_tools.publishText(text_pose, "Go to scan position", rvt::WHITE, rvt::XLARGE);
      visual_tools.trigger();

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");





      // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
      // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
      ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", yi + N_bari_y*xi);

      move_group_interface.attachObject(collision_object_baris[yi + N_bari_y*xi].id, "link_tool");

      visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
      visual_tools.trigger();






      /* Wait for MoveGroup to receive and process the attached collision object message */
      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");





      geometry_msgs::Pose target_pose_final;
      target_pose_final.orientation.x = 1.0;
      target_pose_final.position.x = 0.28 + 0.08*xi;;
      target_pose_final.position.y = +0.35 + 0.1*yi;
      target_pose_final.position.z = 0.0;
      move_group_interface.setPoseTarget(target_pose_final);


      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, " current : show target_pose_final", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishAxisLabeled(target_pose_final, "target_pose_final");
      visual_tools.trigger(); // to apply changes





      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");





      // Replan, but now with the object in hand.
      move_group_interface.setStartStateToCurrentState();



      std::vector<double> traj3 = go_to_position(move_group_interface, target_pose_final, visual_tools, joint_model_group);

      visual_tools.publishText(text_pose, "Go to scan position", rvt::WHITE, rvt::XLARGE);
      visual_tools.trigger();

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");





      // The result may look something like this:
      //
      // .. image:: ./move_group_interface_tutorial_attached_object.gif
      //    :alt: animation showing the arm moving differently once the object is attached
      //
      // Detaching and Removing Objects
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      //
      // Now, let's detach the cylinder from the robot's gripper.
      ROS_INFO_NAMED("tutorial", "Detach the object from the robot : %d", yi + N_bari_y*xi);
      move_group_interface.detachObject(collision_object_baris[yi + N_bari_y*xi].id);

      // Show text in RViz of status
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);
      visual_tools.trigger();


      /* Wait for MoveGroup to receive and process the attached collision object message */
      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");


      
    }
  }

}





  // Now, let's remove the objects from the world.
  ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
  std::vector<std::string> object_ids;

  for (std::vector<moveit_msgs::CollisionObject>::iterator it = collision_object_baris.begin() ; it != collision_object_baris.end(); it++)
  {
    moveit_msgs::CollisionObject obj = *it;
    object_ids.push_back(obj.id);
  }


  //object_ids.push_back(object_to_attach.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();


  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
