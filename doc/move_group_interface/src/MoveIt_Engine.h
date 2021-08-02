#pragma once



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
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>


// TF2

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <extcode.h>
#include "LabVIEW_Types.h"

class MoveIt_Engine

{
public:

  static void CreateInstance();
  static MoveIt_Engine& GetInstance();
  static bool CheckInstance();
  static void ReleaseInstance();
  virtual ~MoveIt_Engine();



  int SetLoggingPath(const LStrHandle InnoViA_log);
  MoveIt_Engine();
  MoveIt_Engine(int argc, char** argv, moveit::planning_interface::MoveGroupInterface move_group_interface,
                moveit::planning_interface::PlanningSceneInterface planning_scene_interface,
                const moveit::core::JointModelGroup* joint_model_group);


  // time spent since the beginning of the program to find a trajectory (not cartesian)
  double Global_time_find_planning = 0;
  // time spent since the beginning of the program to execute a trajectory (not cartesian)
  double Global_time_traj = 0;
  // Number of steps since the beginning of the program in trajectories (not cartesian)
  double Global_nb_step_traj = 0;
  // Distance travelled by the frame in the space of the finger since the beginning of the program  (not cartesian)
  double Global_move_tool = 0;
  // Distance travelled in the config space since the beginning of the program (not cartesian)
  double Global_angle_move_tool = 0;
  // Number of failures to find a path since the beginning of the program (not cartesian)   
  double NB_fail = 0;
  // Number of trajectory found since the beginning of the program (not cartesian)   
  double NUM_TRAJ = 0;

  static const std::string PLANNING_GROUP;

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface* move_group_interface_ptr;

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_ptr;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group_ptr;

  //The package MoveItVisualTools provides many capabilities for visualizing objects, robots, 
  //and trajectories in RViz as well as debugging tools such as step - by - step introspection of a script
  moveit_visual_tools::MoveItVisualTools* visual_tools_ptr;

  //manualy creates an arrow for visualizaton
  void createArrowMarker(visualization_msgs::Marker& marker, const geometry_msgs::Pose& pose,
                         const Eigen::Vector3d& dir, int id, double scale = 0.1);

  //manualy creates frame with tree arrow for visualizaton in a spefic frame
  void createFrameMarkers(visualization_msgs::MarkerArray& markers, const geometry_msgs::PoseStamped& target,
                          const std::string& ns, bool locked = false);

  //return a quaternion in oder to have a random orientation
  tf2::Quaternion random_quaternion();

  //return a quaternion with the rx, ry and rz convention in deg used in halcon (and others) by quentin
  tf2::Quaternion set_quentin(float rx, float ry, float rz);

  // allow to fill a vector with a std::cin cmdn, the info is used to give a information to decribe what are the informations needed
  void fill_vector_cin(std::vector<double>& boxInsideSizeM, std::string info);

  // allow to fill a vector with a std::cin cmdn, the info is used to give a information to decribe what are the informations needed
   void fill_vector_cin_angle(std::vector<double>& boxInsideSizeM, std::string info);

  // return a PoseStamped with a tx, ty, tz, rx, ry, rz (quentin convention) in the frame described by base
  geometry_msgs::PoseStamped get_pose(std::vector<double> vector_quentin, std::string base);

  // load 8 bari for the full scenario 
  void load_bari_in_scene_simulation(std::vector<moveit_msgs::CollisionObject>& collision_object_baris);

  // create a shape from a cao path
  shape_msgs::Mesh load_mesh(std::string modelpath);

  // load a objest in the virtual scene from a shape and a pose, this object need a name : str_id_obj
  void load_obj_in_scene(std::vector<moveit_msgs::CollisionObject>& collision_object_baris, shape_msgs::Mesh mesh_bary,
                         geometry_msgs::PoseStamped pose, std::string str_id_obj);

  // load a carton from a path, this object need a name : str_id_obj
  void load_carton_in_scene(std::vector<moveit_msgs::CollisionObject>& collision_object_baris, std::string modelpath, std::string str_id_obj);

  // load a carton from a path by creating it with its pose in , size and thickness, 
  void create_carton_in_scene(std::vector<moveit_msgs::CollisionObject>& collision_object_baris, std::string frame_base,
                                            std::vector<double> boxInsideSizeM, std::vector<double> boxInsidePoseM,
                                            std::vector<double> boxThickSideBottomM);
  // calcul the distance between two point 3D 
  double distance(double x1, double y1, double z1, double x2, double y2, double z2);

  // calcul the distance between two point 6D
  double distance6D(double x1, double y1, double z1, double v1, double u1, double w1, double x2, double y2, double z2,
                    double v2, double u2, double w2);

  // set the joint constaints for the trajectory, Imapct IK solution and reduce Reasearch space
  void set_joint_constraint(std::vector<float> above_zero, std::vector<float> below_zero);

  // set the config of the planner with the time limit used by each trajectory research, Nb_attempt allows to use multiple thread, 
  // planner_id allow to choose a specific path planing algorithme
  void setup_planner(int Nb_attempt, double time_limit, std::string planner_id);


  // get statistique metrics in order to evaluate the planned trajectory
  EigenSTL::vector_Vector3d evaluate_plan(moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                          double& local_time_find_plan, double& time_traj,
                                          robot_trajectory::RobotTrajectory& trajecto_state);

  //return the pose of the link6/flange grasping pose from tcp grasping pose in quentin's file in the object frame, we can give the distance of the approch with recul 
  geometry_msgs::PoseStamped link6_in_bari_grasp(Eigen::Isometry3d tf_tcp_in_bari, float recul);

  //return the homogeneous matrix of the tcp in bari's frame
  Eigen::Isometry3d create_iso_tcp_in_bari(float tx, float ty, float tz, float rx, float ry, float rz);

  //1st version go to position described with a vector of angle : 1 solution, no inverse kinematics
  std::vector<std::vector<double>> go_to_position(std::vector<double> target_pose);

  //2nd version go to position described by a 3d pose in a specific frame : multiple solutions, inverse kinematics can impact result
  std::vector<std::vector<double>> go_to_position(geometry_msgs::PoseStamped target_pose);

  //3rd version go to position described by a vector of grasping pose and the grasping pose id : multiple solutions, inverse kinematics can impact result
  // if the id is -1 it will give all the vector to the path planning algorithme
  std::vector<std::vector<double>> go_to_position(std::vector<geometry_msgs::PoseStamped> grasp_poses, int ID_grasp);

  //write a trajectory in a file
  void export_trajectory(std::vector<std::vector<double>> traj0);

  //moveit_msgs::RobotTrajectory trajectory to std::vector<std::vector<double>> trajectory
  std::vector<std::vector<double>> MoveIt_Engine::traj_to_vector(moveit_msgs::RobotTrajectory trajectory);

  //version go to position described by a 3d pose in a specific frame solved by cartesian path, 
  // no path planning algorithm used, collisions not checked but can be checked by modifying in the function
  std::vector<std::vector<double>> trajecto_approch(geometry_msgs::PoseStamped& pose_goal);


  // following functions represente all the possible trajectories we can get in innopick, and allow to get path planning and approch
  std::vector<std::vector<double>> trajecto_scan_to_bari(geometry_msgs::PoseStamped bari_pose);

  std::vector<std::vector<double>> trajecto_scan_to_bari(int ID_grasp);


  std::vector<std::vector<double>> trajecto_bari_to_scan(geometry_msgs::PoseStamped scan_pose);

  std::vector<std::vector<double>> trajecto_scan_to_out(geometry_msgs::PoseStamped& final_pose);

  std::vector<std::vector<double>> trajecto_bari_to_scan(geometry_msgs::PoseStamped scan_pose, int ID_grasp);

  std::vector<std::vector<double>> trajecto_out_to_bari(geometry_msgs::PoseStamped& bari_pose);

  std::vector<std::vector<double>> trajecto_out_to_bari(int ID_grasp);

  std::vector<std::vector<double>> trajecto_bari_to_out(int ID_grasp, geometry_msgs::PoseStamped& final_pose);

  std::vector<std::vector<double>> trajecto_bari_to_out(geometry_msgs::PoseStamped& final_pose);

  // translation(m) / Rotation (deg)
  // Poses defined in object's frame (x,y,z,rx,ry,rz)
  // load the grasping pose from the file, we can give the distance of the approch with recul
  int load_grasping_pose(float distance_approch, std::string FilePath);

  //do the scenario of 16 trajectories used in evaluating th path planning, with M = 2, N = 2
  // M is the number of cycle, N is the number of obect detected by 1 scan
  void full_scenario_grasp_robot(int M, int N);

  // do sceneario but ask at all step the informations with a std::cin, used to test with the true robot where we don't in advance the information
  void full_scenario_grasp(int M, int N, std::string planni_id);


  // double vector where we get the grasping pose in the objcet's frame
  // vec_grasping_poses[0] are the true grasping poses 
  // vec_grasping_poses[1] are the grasping poses before approch
  std::vector<std::vector<geometry_msgs::PoseStamped>> vec_grasping_poses;

  // used to save the approch traj
  std::vector<std::vector<double>> approch_traj;

  // used to save the approch traj
  moveit_msgs::RobotTrajectory approch_traj_msg;

  // used to save the apprch traj inverse for the return
  moveit_msgs::RobotTrajectory inv_approch_msg_traj;

  //bool to execute or not the approch
  bool do_execution_approch = 1;

 private:
  	//*************************************
  //  Errors and Logs handle

  static std::ofstream *out, *err;
  static std::streambuf *coutbuf, *cerrbuf;

  //*************************************
  //  Instance handle

  static MoveIt_Engine* instance;

  MoveIt_Engine(const MoveIt_Engine&) = delete;
  MoveIt_Engine& operator=(const MoveIt_Engine&) = delete;
  MoveIt_Engine(MoveIt_Engine&&) = delete;
  MoveIt_Engine& operator=(MoveIt_Engine&&) = delete;


};


