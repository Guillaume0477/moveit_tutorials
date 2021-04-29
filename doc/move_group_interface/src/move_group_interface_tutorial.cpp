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
#include <thread>


// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

double Global_time_find_planning = 0;
double Global_time_traj = 0;
double Global_nb_step_traj = 0;
double Global_move_tool = 0;
double NUM_TRAJ = 0;


void load_bari_in_scene(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, const Eigen::Vector3d center){

  //std::string modelpath = "/home/pc-m/Documents/My-cao/barillet.obj";
  std::string modelpath = "package://geometric_shapes/test/resources/barillet_convex.obj";
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

  // shape_msgs::SolidPrimitive work_box;
  // work_box.type = work_box.BOX;
  // work_box.dimensions.resize(3);
  // work_box.dimensions[work_box.BOX_X] = 0.60; 
  // work_box.dimensions[work_box.BOX_Y] = 1.0; 
  // work_box.dimensions[work_box.BOX_Z] = 0.9; 

  // geometry_msgs::Pose work_box_pose;
  // work_box_pose.orientation.w = 1.0;
  // work_box_pose.position.x = 0.5;
  // work_box_pose.position.y = 0.155;
  // work_box_pose.position.z = 0.45;


  // collision_object_bari.primitives.push_back(work_box);
  // collision_object_bari.primitive_poses.push_back(work_box_pose);
  // collision_object_bari.operation = collision_object_bari.ADD;

  collision_object_baris.push_back(collision_object_bari);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_object_baris);


}


double distance(double x1, double y1, double z1, double x2, double y2, double z2){

  return sqrt(pow(x1-x2,2)+pow(y1-y2,2)+pow(z1-z2,2));

}

std::vector<double> go_to_position_begin(moveit::planning_interface::MoveGroupInterface &move_group_interface , geometry_msgs::Pose target_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group){
  

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
  move_group_interface.setPlanningTime(50);
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
  std::string pipeline_test = move_group_interface.getPlanningPipelineId();
  std::cout<<"getPlannerId : "<<planner_test<<std::endl;
  std::cout<<"getPipeline : "<<pipeline_test<<std::endl;

  //std::cout<<planner_test*<<std::endl;



  move_group_interface.setPoseTarget(target_pose);
  // /** \brief Get the current joint state goal in a form compatible to setJointValueTarget() */
  // void getJointValueTarget(std::vector<double>& group_variable_values) const;



  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


  ROS_INFO_NAMED("tutorial", " Go to scan position %s", success ? "" : "FAILED");

  double time_find_plan = my_plan.planning_time_;

  std::cout<< "TIME TO FIND THE PATH : " << time_find_plan <<std::endl;

  std::chrono::high_resolution_clock::time_point begin_traj = std::chrono::high_resolution_clock::now();
  move_group_interface.execute(my_plan);
  std::chrono::high_resolution_clock::time_point end_traj = std::chrono::high_resolution_clock::now();

  double time_traj = std::chrono::duration_cast<std::chrono::microseconds>(end_traj-begin_traj).count()/1000000.0;

  std::cout<< "TIME TO EXECUTE TRAJ : " << time_traj <<std::endl;





  robot_trajectory::RobotTrajectory trajecto_state(move_group_interface.getCurrentState()->getRobotModel(), move_group_interface.getName());

  trajecto_state.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), my_plan.trajectory_);

  double size_traj = trajecto_state.getWayPointCount();

  ROS_INFO_NAMED("trajectory print : ", " test size : %f ", size_traj  );

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  trajecto_state.getWayPointPtr(0)->copyJointGroupPositions(joint_model_group, joint_group_positions);
  int number_joint = 7;

  for (int i =0 ; i< number_joint  ; i++ )
  {
    std::cout << "angle joint i = " << i << " : " << joint_group_positions[i]*180/3.14159265 << std::endl;
  }
  //std::cout << "test std::cout" << trajecto_state.getWayPoint(0).getVariablePositions << std::endl;  //trajecto_state.getWayPointCount()
  std::cout << "test std::cout" << trajecto_state.getWayPoint(0) << std::endl;  //trajecto_state.getWayPointCount()

  std::cout << "Mine:" << std::endl;
  std::cout << "Mine:" << std::endl;
  std::cout << "Mine:" << std::endl;

  //std::cout << "Link poses:" << std::endl;
 
  for (std::size_t i = 1; i < trajecto_state.getWayPointCount(); ++i)
  {
    const Eigen::Isometry3d transform0 =trajecto_state.getWayPoint(i-1).getGlobalLinkTransform("link_finger1");
    const Eigen::Isometry3d transform1 =trajecto_state.getWayPoint(i).getGlobalLinkTransform("link_finger1");

    ASSERT_ISOMETRY(transform0)  // unsanitized input, could contain a non-isometry
    ASSERT_ISOMETRY(transform1)
    //Eigen::Quaterniond q(transform.linear());
    //std::cout << "T.xyz = [" << transform.translation().x() << ", " << transform.translation().y() << ", "
    //<< transform.translation().z() << "], Q.xyzw = [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w()
    //<< "]" << std::endl;

    //std::cout << "TYPE!! : "<< typeid(transform.translation()).name() << std::endl;

    //double dist = distance((double) transform.translation().x(),(double) transform.translation().y(),(double) transform.translation().z(),(double) transform.translation().x(),(double) transform.translation().y(),(double) transform.translation().z()); 
    double dist = distance((double) transform0.translation().x(),(double) transform0.translation().y(),(double) transform0.translation().z(),(double) transform1.translation().x(),(double) transform1.translation().y(),(double) transform1.translation().z()); 


  }
  return joint_group_positions;
}

void setup_planner_constrain(moveit::planning_interface::MoveGroupInterface &move_group_interface){

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



  std::string planner_id = "RRTConnect";
  move_group_interface.setPlannerId(planner_id);
  std::map<std::string, std::string> parametre = move_group_interface.getPlannerParams(planner_id,"arm_group");

  std::map<std::string, std::string>::iterator it;
  for (it = parametre.begin(); it != parametre.end(); it++)
  {
    std::cout << it->first    // string (key)
    << " : "
    << it->second   // string's value 
    << std::endl;
  }
  std::cout << "parametre[type] : "<< parametre["type"] << std::endl;
  //parametre["type"] = "geometric::LazyPRMstar";
  //std::cout << "parametre[type] : "<< parametre["type"] << std::endl;

  //move_group_interface.setPlannerParams(planner_id, "arm_group", parametre);


  move_group_interface.setPlanningTime(300);
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






  //std::cout<<planner_test*<<std::endl;


  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = "link_tool";
  pcm.header.frame_id = "base_link";
  
  shape_msgs::SolidPrimitive work_box;
  work_box.type = work_box.BOX;
  work_box.dimensions.resize(3);
  work_box.dimensions[work_box.BOX_X] = 0.60; 
  work_box.dimensions[work_box.BOX_Y] = 1.0; 
  work_box.dimensions[work_box.BOX_Z] = 0.9; 

  geometry_msgs::Pose work_box_pose;
  work_box_pose.orientation.w = 1.0;
  work_box_pose.position.x = 0.5;
  work_box_pose.position.y = 0.155;
  work_box_pose.position.z = 0.45;

  pcm.constraint_region.primitives.push_back(work_box);
  pcm.constraint_region.primitive_poses.push_back(work_box_pose);
  pcm.weight = 1.0;

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "link_6";
  ocm.header.frame_id = "base_link";
  ocm.orientation.x = 0.93373;
  ocm.orientation.y = -0.35765;
  ocm.orientation.z = 0.0057657;
  ocm.orientation.w = 0.014457;
  ocm.absolute_x_axis_tolerance = 2.0; //90 degre abs
  ocm.absolute_y_axis_tolerance = 2.0;
  ocm.absolute_z_axis_tolerance = 10.0;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  test_constraints.position_constraints.push_back(pcm);
  move_group_interface.setPathConstraints(test_constraints);


}



void setup_planner(moveit::planning_interface::MoveGroupInterface &move_group_interface){

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



  std::string planner_id = "RRTConnect";
  move_group_interface.setPlannerId(planner_id);
  std::map<std::string, std::string> parametre = move_group_interface.getPlannerParams(planner_id,"arm_group");

  std::map<std::string, std::string>::iterator it;
  for (it = parametre.begin(); it != parametre.end(); it++)
  {
    std::cout << it->first    // string (key)
    << " : "
    << it->second   // string's value 
    << std::endl;
  }
  std::cout << "parametre[type] : "<< parametre["type"] << std::endl;
  //parametre["type"] = "geometric::LazyPRMstar";
  //std::cout << "parametre[type] : "<< parametre["type"] << std::endl;

  //move_group_interface.setPlannerParams(planner_id, "arm_group", parametre);


  move_group_interface.setPlanningTime(300);
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






  //std::cout<<planner_test*<<std::endl;


  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = "link_tool";
  pcm.header.frame_id = "base_link";
  
  shape_msgs::SolidPrimitive work_box;
  work_box.type = work_box.BOX;
  work_box.dimensions.resize(3);
  work_box.dimensions[work_box.BOX_X] = 0.60; 
  work_box.dimensions[work_box.BOX_Y] = 1.0; 
  work_box.dimensions[work_box.BOX_Z] = 0.9; 

  geometry_msgs::Pose work_box_pose;
  work_box_pose.orientation.w = 1.0;
  work_box_pose.position.x = 0.5;
  work_box_pose.position.y = 0.155;
  work_box_pose.position.z = 0.45;

  pcm.constraint_region.primitives.push_back(work_box);
  pcm.constraint_region.primitive_poses.push_back(work_box_pose);
  pcm.weight = 1.0;

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "link_6";
  ocm.header.frame_id = "base_link";
  ocm.orientation.x = 0.93373;
  ocm.orientation.y = -0.35765;
  ocm.orientation.z = 0.0057657;
  ocm.orientation.w = 0.014457;
  ocm.absolute_x_axis_tolerance = 2.0; //90 degre abs
  ocm.absolute_y_axis_tolerance = 2.0;
  ocm.absolute_z_axis_tolerance = 10.0;
  ocm.weight = 1.0;

  //moveit_msgs::Constraints test_constraints;
  //test_constraints.orientation_constraints.push_back(ocm);
  //test_constraints.position_constraints.push_back(pcm);
  //move_group_interface.setPathConstraints(test_constraints);


}

EigenSTL::vector_Vector3d evaluate_plan(moveit::planning_interface::MoveGroupInterface::Plan &plan,double &local_time_find_plan, double &time_traj, robot_trajectory::RobotTrajectory &trajecto_state, moveit_visual_tools::MoveItVisualTools &visual_tools){



  Global_time_find_planning += local_time_find_plan;

  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<< "TIME TO FIND THE PATH " << NUM_TRAJ << " : " << local_time_find_plan << " s" << std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;



  Global_time_traj += time_traj;


  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<< "TIME TO EXECUTE TRAJ " << NUM_TRAJ << " : " << time_traj << " s" <<std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;




  double size_traj = trajecto_state.getWayPointCount();

  ROS_INFO_NAMED("trajectory print : ", " test size : %f ", size_traj  );

  Global_nb_step_traj += size_traj;

  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<< "NB STEP TRAJ " << NUM_TRAJ << " : " << size_traj <<std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;


  // Next get the current set of joint values for the group.
  //std::vector<double> joint_group_positions;
  // trajecto_state.getWayPointPtr(0)->copyJointGroupPositions(joint_model_group, joint_group_positions);
  // int number_joint = 7;

  // for (int i =0 ; i< number_joint  ; i++ )
  // {
  //   std::cout << "angle joint i = " << i << " : " << joint_group_positions[i]*180/3.14159265 << std::endl;
  // }
  // //std::cout << "test std::cout" << trajecto_state.getWayPoint(0).getVariablePositions << std::endl;  //trajecto_state.getWayPointCount()
  // std::cout << "test std::cout" << trajecto_state.getWayPoint(0) << std::endl;  //trajecto_state.getWayPointCount()

  std::cout << "Mine:" << std::endl;
  std::cout << "Mine:" << std::endl;
  std::cout << "Mine:" << std::endl;

  //std::cout << "Link poses:" << std::endl;

  EigenSTL::vector_Vector3d path;

  double local_move_tool = 0;
 
  for (std::size_t i = 1; i < trajecto_state.getWayPointCount(); ++i)
  {
    const Eigen::Isometry3d transform0 =trajecto_state.getWayPoint(i-1).getGlobalLinkTransform("link_finger1");
    const Eigen::Isometry3d transform1 =trajecto_state.getWayPoint(i).getGlobalLinkTransform("link_finger1");

    ASSERT_ISOMETRY(transform0)  // unsanitized input, could contain a non-isometry
    ASSERT_ISOMETRY(transform1)

    path.push_back(transform0.translation());
    visual_tools.publishSphere(transform0, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
    //Eigen::Quaterniond q(transform.linear());
    //std::cout << "T.xyz = [" << transform.translation().x() << ", " << transform.translation().y() << ", "
    //<< transform.translation().z() << "], Q.xyzw = [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w()
    //<< "]" << std::endl;

    //std::cout << "TYPE!! : "<< typeid(transform.translation()).name() << std::endl;

    //double dist = distance((double) transform.translation().x(),(double) transform.translation().y(),(double) transform.translation().z(),(double) transform.translation().x(),(double) transform.translation().y(),(double) transform.translation().z()); 
    double dist = distance((double) transform0.translation().x(),(double) transform0.translation().y(),(double) transform0.translation().z(),(double) transform1.translation().x(),(double) transform1.translation().y(),(double) transform1.translation().z()); 
    Global_move_tool += dist;
    local_move_tool += dist;


  }


  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<< "DIST TOOL FOR TRAJ " << NUM_TRAJ << " : " << local_move_tool << "m" <<std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;


  return path;

}


std::vector<std::vector<double>> go_to_position(moveit::planning_interface::MoveGroupInterface &move_group_interface ,geometry_msgs::Pose target_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group){


  NUM_TRAJ += 1;


  move_group_interface.setPoseTarget(target_pose);
  // /** \brief Get the current joint state goal in a form compatible to setJointValueTarget() */
  // void getJointValueTarget(std::vector<double>& group_variable_values) const;


  std::string planner_test = move_group_interface.getPlannerId();
  std::cout<<"getPlannerId : "<<planner_test<<std::endl;


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  double local_time_find_plan = 0;
  bool success = false;
  for (int i=0; i < 10 ;i++){
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
      local_time_find_plan += my_plan.planning_time_;
      break;
    }
    else {
      local_time_find_plan += move_group_interface.getPlanningTime();
      std::cout << "TRY AGAIN : " << i << std::endl;
    }

  }

  if (!success) {
    std::cerr << "FAIL TO FINF A PATH AFTER 10 TRY AGAIN" << std::endl;
    exit(0);
  }
  else{

  }


  ROS_INFO_NAMED("tutorial", " Go to scan position %s", success ? "" : "FAILED");

  std::chrono::high_resolution_clock::time_point begin_traj = std::chrono::high_resolution_clock::now();
  move_group_interface.execute(my_plan);
  std::chrono::high_resolution_clock::time_point end_traj = std::chrono::high_resolution_clock::now();

  double time_traj = std::chrono::duration_cast<std::chrono::microseconds>(end_traj-begin_traj).count()/1000000.0;

  robot_trajectory::RobotTrajectory trajecto_state(move_group_interface.getCurrentState()->getRobotModel(), move_group_interface.getName());

  trajecto_state.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), my_plan.trajectory_);

  std::vector<std::vector<double>> trajecto_waypoint_joint;
  for (std::size_t i = 1; i < trajecto_state.getWayPointCount(); ++i)
  {

    std::vector<double> waypoint_joint;
    trajecto_state.getWayPointPtr(i)->copyJointGroupPositions(joint_model_group, waypoint_joint);
    trajecto_waypoint_joint.push_back(waypoint_joint);
  }





  EigenSTL::vector_Vector3d path = evaluate_plan(my_plan, local_time_find_plan, time_traj, trajecto_state, visual_tools);

  const double radius = 0.005;
  visual_tools.publishPath(path, rviz_visual_tools::GREEN, radius);


  return trajecto_waypoint_joint;
}



void trajecto_scan_to_bari(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::Pose scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, geometry_msgs::Pose bari_pose){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();

  std::vector<std::vector<double>> traj6 = go_to_position(move_group_interface, bari_pose, visual_tools, joint_model_group);

  visual_tools.publishText(text_pose, "Go to scan position", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

}



void trajecto_intial_to_bari(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::Pose scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<geometry_msgs::Pose> &bari_poses){

  move_group_interface.setStartState(*move_group_interface.getCurrentState());


  std::vector<double> traj1 = go_to_position_begin(move_group_interface, scan_pose, visual_tools, joint_model_group);

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  visual_tools.publishText(text_pose, "Go to scan position", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  const Eigen::Vector3d center(0.5,0.0,0.0);
  //center.x;

  load_bari_in_scene(planning_scene_interface, move_group_interface, collision_object_baris, center);
  load_carton_in_scene(planning_scene_interface, move_group_interface, collision_object_baris, center);


  std::chrono::seconds dura70(70);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  //ROS_INFO_NAMED("tutorial", "Add an object into the world");
  //planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  visual_tools.publishText(text_pose, "Add object", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");




  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, " current : show bari position", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.publishAxisLabeled(bari_poses[0], "bari_pose");
  visual_tools.trigger(); // to apply changes





  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");




  move_group_interface.setStartState(*move_group_interface.getCurrentState());


  trajecto_scan_to_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses[0]);

}



void trajecto_bari_to_scan(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::Pose scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();

  std::vector<std::vector<double>> traj6 = go_to_position(move_group_interface, scan_pose, visual_tools, joint_model_group);

  visual_tools.publishText(text_pose, "Go to scan position", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

}


void trajecto_scan_to_out(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::Pose scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, geometry_msgs::Pose &final_pose){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();

  std::vector<std::vector<double>> traj6 = go_to_position(move_group_interface, final_pose, visual_tools, joint_model_group);

  visual_tools.publishText(text_pose, "Go to scan position", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

}



void trajecto_bari_to_out_by_scan(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::Pose scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, geometry_msgs::Pose &final_pose){


  trajecto_bari_to_scan(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group);

  move_group_interface.setStartState(*move_group_interface.getCurrentState());

  trajecto_scan_to_out(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, final_pose);

}

void trajecto_out_to_bari(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::Pose scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, geometry_msgs::Pose &bari_pose){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();

  std::vector<std::vector<double>> traj5 = go_to_position(move_group_interface, bari_pose, visual_tools, joint_model_group);

  visual_tools.publishText(text_pose, "Go to scan position", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


}


void trajecto_bari_to_out(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::Pose scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, geometry_msgs::Pose &final_pose){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();

  std::vector<std::vector<double>> traj6 = go_to_position(move_group_interface, final_pose, visual_tools, joint_model_group);

  visual_tools.publishText(text_pose, "Go to scan position", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

}


void full_scenario(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::Pose scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<geometry_msgs::Pose> &final_poses, std::vector<geometry_msgs::Pose> &bari_poses, int M, int N){

  //#####
  //##### begin scenario #####
  //#####

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  
  trajecto_intial_to_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses);


  setup_planner(move_group_interface);



  for (int i = 0; i < M; i++){

    // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
    // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
    ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1));

    move_group_interface.attachObject(collision_object_baris[i*(N+1)].id, "link_tool");

    move_group_interface.setStartState(*move_group_interface.getCurrentState());

    trajecto_bari_to_out_by_scan(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, final_poses[i*(N+1)]);


    // The result may look something like this:
    //
    // .. image:: ./move_group_interface_tutorial_attached_object.gif
    //    :alt: animation showing the arm moving differently once the object is attached
    //
    // Detaching and Removing Objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Now, let's detach the cylinder from the robot's gripper.
    ROS_INFO_NAMED("tutorial", "Detach the object from the robot : %d", i*(N+1));
    move_group_interface.detachObject(collision_object_baris[i*(N+1)].id);

    // Show text in RViz of status
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Object detached from robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.trigger();


    /* Wait for MoveGroup to receive and process the attached collision object message */
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");



    for (int j = 0; j< N; j++){


      // geometry_msgs::Pose bari_pose;
      // bari_pose.orientation.x = 0.93373;
      // bari_pose.orientation.y = -0.35765;
      // bari_pose.orientation.z = 0.0057657;
      // bari_pose.orientation.w = 0.014457;



      // bari_pose.position.x = 0.5+ 0.04*xi;
      // bari_pose.position.y = -0.05 -0.04+ 0.05*yi;
      // bari_pose.position.z = 0.53731+0.02;

      move_group_interface.setStartState(*move_group_interface.getCurrentState());




      trajecto_out_to_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses[i*(N+1) + j + 1]);



      // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
      // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
      ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1) + j + 1);

      move_group_interface.attachObject(collision_object_baris[i*(N+1) + j + 1].id, "link_tool");

      visual_tools.publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      visual_tools.trigger();

      //std::chrono::seconds dura10(10);







      /* Wait for MoveGroup to receive and process the attached collision object message */
      //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");





      // geometry_msgs::Pose target_pose_final;
      // target_pose_final.orientation.x = 0.93373;
      // target_pose_final.orientation.y = -0.35765;
      // target_pose_final.orientation.z = 0.0057657;
      // target_pose_final.orientation.w = 0.014457;
      // target_pose_final.position.x = 0.45 + 0.08*xi;;
      // target_pose_final.position.y = +0.35 + 0.1*yi;
      // target_pose_final.position.z = 0.5;
      // move_group_interface.setPoseTarget(target_pose_final);


      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, " current : show target_pose_final", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      visual_tools.publishAxisLabeled(final_poses[i*(N+1) + j + 1], "target_pose_final");
      visual_tools.trigger(); // to apply changes





      //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");





      // Replan, but now with the object in hand.
      move_group_interface.setStartStateToCurrentState();

      trajecto_bari_to_out(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, final_poses[i*(N+1) + j + 1]);


      // The result may look something like this:
      //
      // .. image:: ./move_group_interface_tutorial_attached_object.gif
      //    :alt: animation showing the arm moving differently once the object is attached
      //
      // Detaching and Removing Objects
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      //
      // Now, let's detach the cylinder from the robot's gripper.
      ROS_INFO_NAMED("tutorial", "Detach the object from the robot : %d", i*(N+1) + j + 1);
      move_group_interface.detachObject(collision_object_baris[i*(N+1) + j + 1].id);

      // Show text in RViz of status
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Object detached from robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      visual_tools.trigger();


      /* Wait for MoveGroup to receive and process the attached collision object message */
      //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");



    }



    move_group_interface.setStartState(*move_group_interface.getCurrentState());




    trajecto_out_to_bari(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses[i*(N+1) + N + 1]);

    // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
    // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
    ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1) + N + 1);

    move_group_interface.attachObject(collision_object_baris[i*(N+1) + N + 1].id, "link_tool");

    visual_tools.publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.trigger();

    //std::chrono::seconds dura10(10);







    /* Wait for MoveGroup to receive and process the attached collision object message */
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");





    // geometry_msgs::Pose target_pose_final;
    // target_pose_final.orientation.x = 0.93373;
    // target_pose_final.orientation.y = -0.35765;
    // target_pose_final.orientation.z = 0.0057657;
    // target_pose_final.orientation.w = 0.014457;
    // target_pose_final.position.x = 0.45 + 0.08*xi;;
    // target_pose_final.position.y = +0.35 + 0.1*yi;
    // target_pose_final.position.z = 0.5;
    // move_group_interface.setPoseTarget(target_pose_final);


    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, " current : show target_pose_final", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.publishAxisLabeled(final_poses[i*(N+1) + N + 1], "target_pose_final");
    visual_tools.trigger(); // to apply changes





    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  }



  // Replan, but now with the object in hand.
  move_group_interface.setStartStateToCurrentState();


  trajecto_bari_to_out(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, final_poses[(M-1)*(N+1) + N + 1]);


  // The result may look something like this:
  //
  // .. image:: ./move_group_interface_tutorial_attached_object.gif
  //    :alt: animation showing the arm moving differently once the object is attached
  //
  // Detaching and Removing Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Now, let's detach the cylinder from the robot's gripper.
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot : %d", (M-1)*(N+1) + N + 1);
  move_group_interface.detachObject(collision_object_baris[(M-1)*(N+1) + N + 1].id);

  // Show text in RViz of status
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Object detached from robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();


  /* Wait for MoveGroup to receive and process the attached collision object message */
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");






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
  visual_tools.publishText(text_pose, "Objects removed", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();


}



void full_scenario_without_attach(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::Pose scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<geometry_msgs::Pose> &final_poses, std::vector<geometry_msgs::Pose> &bari_poses, int M, int N){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  std::chrono::seconds dura70(70);

  trajecto_intial_to_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses);
  setup_planner(move_group_interface);

  for (int i = 0; i < M; i++){

    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    trajecto_bari_to_out_by_scan(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, final_poses[i*(N+1)]);

    for (int j = 0; j< N; j++){

      move_group_interface.setStartState(*move_group_interface.getCurrentState());
      trajecto_out_to_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses[i*(N+1) + j + 1]);


      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, " current : show target_pose_final", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      visual_tools.publishAxisLabeled(final_poses[i*(N+1) + j + 1], "target_pose_final");
      visual_tools.trigger(); // to apply changes


      // Replan, but now with the object in hand.
      move_group_interface.setStartStateToCurrentState();
      trajecto_bari_to_out(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, final_poses[i*(N+1) + j + 1]);

    }



    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    trajecto_out_to_bari(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses[i*(N+1) + N + 1]);

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, " current : show target_pose_final", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.publishAxisLabeled(final_poses[i*(N+1) + N + 1], "target_pose_final");
    visual_tools.trigger(); // to apply changes

  }

  // Replan, but now with the object in hand.
  move_group_interface.setStartStateToCurrentState();
  trajecto_bari_to_out(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, final_poses[(M-1)*(N+1) + N + 1]);

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
  visual_tools.publishText(text_pose, "Objects removed", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

}




void scenario_test_1(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::Pose scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<geometry_msgs::Pose> &final_poses, std::vector<geometry_msgs::Pose> &bari_poses, int M, int N){

  //#####
  //##### begin scenario #####
  //#####

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  
  trajecto_intial_to_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses);


  setup_planner(move_group_interface);

  // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
  // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", 0);

  move_group_interface.attachObject(collision_object_baris[0].id, "link_tool");


  for (int i = 0; i < 10; i++){

    move_group_interface.setStartState(*move_group_interface.getCurrentState());

    trajecto_bari_to_scan(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group);

    move_group_interface.setStartState(*move_group_interface.getCurrentState());

    trajecto_scan_to_bari(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses[0]);


  }



  // The result may look something like this:
  //
  // .. image:: ./move_group_interface_tutorial_attached_object.gif
  //    :alt: animation showing the arm moving differently once the object is attached
  //
  // Detaching and Removing Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Now, let's detach the cylinder from the robot's gripper.
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot : %d", (M-1)*(N+1) + N + 1);
  move_group_interface.detachObject(collision_object_baris[(M-1)*(N+1) + N + 1].id);

  // Show text in RViz of status
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Object detached from robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();


  /* Wait for MoveGroup to receive and process the attached collision object message */
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");






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
  visual_tools.publishText(text_pose, "Objects removed", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();


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

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // Adding objects to the environment
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // First let's plan to another simple goal with no objects in the way.

  geometry_msgs::Pose scan_pose;
  // scan_pose.orientation.x = 0.93373;
  // scan_pose.orientation.y = -0.35765;
  // scan_pose.orientation.z = 0.0057657;
  // scan_pose.orientation.w = 0.014457;
  // scan_pose.position.x = 0.75;
  // scan_pose.position.y = 0.0;
  // scan_pose.position.z = 0.5;
  scan_pose.orientation.x = -0.65663;
  scan_pose.orientation.y = 0.25469;
  scan_pose.orientation.z = 0.25726;
  scan_pose.orientation.w = 0.66166;
  scan_pose.position.x = 0.48;
  scan_pose.position.y = -0.05;
  scan_pose.position.z = 0.51;


  std::vector<geometry_msgs::Pose> bari_poses;
  std::vector<geometry_msgs::Pose> final_poses;


  int N_bari_x = 2;
  int N_bari_y = 4;
  int N_bari_z = 1;
  for (int xi = 0; xi< N_bari_x;xi++){
    for (int yi = 0; yi< N_bari_y;yi++){
      for (int zi = 0; zi< N_bari_z;zi++){


        geometry_msgs::Pose target_pose_final;
        target_pose_final.orientation.x = 0.93373;
        target_pose_final.orientation.y = -0.35765;
        target_pose_final.orientation.z = 0.0057657;
        target_pose_final.orientation.w = 0.014457;
        target_pose_final.position.x = 0.45 + 0.08*xi;;
        target_pose_final.position.y = +0.35 + 0.1*yi;
        target_pose_final.position.z = 0.5;
        // target_pose_final.orientation.x = -0.65663;
        // target_pose_final.orientation.y = 0.25469;
        // target_pose_final.orientation.z = 0.25726;
        // target_pose_final.orientation.w = 0.66166;
        // target_pose_final.position.x = 0.48;
        // target_pose_final.position.y = -0.05;
        // target_pose_final.position.z = 0.51;

        final_poses.push_back(target_pose_final);


        geometry_msgs::Pose bari_pose;
        bari_pose.orientation.x = 0.93373;
        bari_pose.orientation.y = -0.35765;
        bari_pose.orientation.z = 0.0057657;
        bari_pose.orientation.w = 0.014457;
        bari_pose.position.x = 0.5+ 0.04*xi;
        bari_pose.position.y = -0.05 -0.04+ 0.05*yi;
        bari_pose.position.z = 0.53731+0.02;
        // bari_pose.orientation.x = 0.93373;
        // bari_pose.orientation.y = -0.35765;
        // bari_pose.orientation.z = 0.0057657;
        // bari_pose.orientation.w = 0.014457;
        // bari_pose.position.x = 0.5;
        // bari_pose.position.y = -0.05 -0.04;
        // bari_pose.position.z = 0.53731+0.02;
    
        bari_poses.push_back(bari_pose);

      }
    }
  }


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  std::vector<moveit_msgs::CollisionObject> collision_object_baris;

  int M = 2;
  int N = 2; //(+1 bari detecté par scan)



  //full_scenario_without_attach( move_group_interface, collision_object_baris, planning_scene_interface, scan_pose, visual_tools, joint_model_group, final_poses, bari_poses, M, N);

  //full_scenario( move_group_interface, collision_object_baris, planning_scene_interface, scan_pose, visual_tools, joint_model_group, final_poses, bari_poses, M, N);


  scenario_test_1( move_group_interface, collision_object_baris, planning_scene_interface, scan_pose, visual_tools, joint_model_group, final_poses, bari_poses, M, N);


  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<<"Global time to find plan : "<<Global_time_find_planning<<" s "<<std::endl;
  std::cout<<"Global time to execute traj : "<<Global_time_traj<<" s "<<std::endl;
  std::cout<<"Global nb of step in the traj : "<<Global_nb_step_traj<<std::endl;
  std::cout<<"Global move of the tool : "<<Global_move_tool<< " m"<<std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;

  /* Wait for MoveGroup to receive and process the attached collision object message */
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");


  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
