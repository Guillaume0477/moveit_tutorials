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

  double Global_time_find_planning = 0;
  double Global_time_traj = 0;
  double Global_nb_step_traj = 0;
  double Global_move_tool = 0;
  double Global_angle_move_tool = 0;
  double NB_fail = 0;
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

  moveit_visual_tools::MoveItVisualTools* visual_tools_ptr;

  void createArrowMarker(visualization_msgs::Marker& marker, const geometry_msgs::Pose& pose,
                         const Eigen::Vector3d& dir, int id, double scale = 0.1);


  void createFrameMarkers(visualization_msgs::MarkerArray& markers, const geometry_msgs::PoseStamped& target,
                          const std::string& ns, bool locked = false);

  tf2::Quaternion random_quaternion();

  tf2::Quaternion set_quentin(float rx, float ry, float rz);

  void fill_vector_cin(std::vector<double>& boxInsideSizeM, std::string info);

  void fill_vector_cin_angle(std::vector<double>& boxInsideSizeM, std::string info);

  geometry_msgs::PoseStamped get_pose(std::string info);

  void load_bari_in_scene_simulation(std::vector<moveit_msgs::CollisionObject>& collision_object_baris);

  shape_msgs::Mesh load_mesh(std::string modelpath);

  void load_obj_in_scene(std::vector<moveit_msgs::CollisionObject>& collision_object_baris, shape_msgs::Mesh mesh_bary,
                         geometry_msgs::PoseStamped pose, std::string str_id_obj);

  void load_carton_in_scene(std::vector<moveit_msgs::CollisionObject>& collision_object_baris);

  void create_carton_in_scene(std::vector<moveit_msgs::CollisionObject>& collision_object_baris,
                              std::vector<double> boxInsideSizeM, std::vector<double> boxInsidePoseM,
                              std::vector<double> boxThickSideBottomM);

  double distance(double x1, double y1, double z1, double x2, double y2, double z2);

  double distance6D(double x1, double y1, double z1, double v1, double u1, double w1, double x2, double y2, double z2,
                    double v2, double u2, double w2);

  bool checkIkValidity(robot_state::RobotState* robot_state, const robot_state::JointModelGroup* joint_group,
                       const double* joint_group_variable_values);

  geometry_msgs::Quaternion apply_rotation(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q_to_apply);

  tf2::Quaternion apply_rotation(tf2::Quaternion q1, tf2::Quaternion q_rot);

  void set_joint_constraint(std::vector<float> above_zero, std::vector<float> below_zero);

  // moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(&isIKStateValid, static_cast<const
  // planning_scene::PlanningSceneConstPtr&>(*ls).get(),
  //                           collision_checking_verbose_, only_check_self_collision_, visual_tools_, _1, _2, _3);

  void setup_planner(int Nb_attempt, double time_limit, std::string planner_id);

  EigenSTL::vector_Vector3d evaluate_plan(moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                          double& local_time_find_plan, double& time_traj,
                                          robot_trajectory::RobotTrajectory& trajecto_state);

  geometry_msgs::PoseStamped link6_in_bari_grasp(Eigen::Isometry3d tf_tcp_in_bari, float recul);

  // translation(m) / Rotation (deg)
  // Poses defined in object's frame (x,y,z,rx,ry,rz)
  Eigen::Isometry3d create_iso_tcp_in_bari(float tx, float ty, float tz, float rx, float ry, float rz);

  std::vector<std::vector<double>> go_to_position(std::vector<double> target_pose);

  std::vector<std::vector<double>> go_to_position(geometry_msgs::PoseStamped target_pose);

  std::vector<std::vector<double>> go_to_position(std::vector<geometry_msgs::PoseStamped> grasp_poses, int ID_grasp);

  void export_trajectory(std::vector<std::vector<double>> traj0);

  std::vector<std::vector<double>> trajecto_approch(geometry_msgs::PoseStamped& pose_goal);

  void trajecto_scan_to_bari(geometry_msgs::PoseStamped bari_pose);

  void trajecto_scan_to_bari(std::vector<std::vector<geometry_msgs::PoseStamped>> vec_grasping_poses, int ID_grasp);

  void trajecto_initial_to_scan_and_bari(std::vector<moveit_msgs::CollisionObject>& collision_object_baris,
                                         geometry_msgs::PoseStamped scan_pose,
                                         std::vector<geometry_msgs::PoseStamped>& bari_poses);

  std::vector<std::vector<double>> trajecto_bari_to_scan(geometry_msgs::PoseStamped scan_pose);

  std::vector<std::vector<double>> trajecto_scan_to_out(geometry_msgs::PoseStamped& final_pose);

  std::vector<std::vector<double>>
  trajecto_bari_to_scan(geometry_msgs::PoseStamped scan_pose,
                        std::vector<std::vector<geometry_msgs::PoseStamped>>& vec_grasping_poses, int ID_grasp);

  void trajecto_out_to_bari(geometry_msgs::PoseStamped& bari_pose);

  void trajecto_out_to_bari(std::vector<std::vector<geometry_msgs::PoseStamped>>& vec_grasping_poses, int ID_grasp);

  void trajecto_bari_to_out(std::vector<std::vector<geometry_msgs::PoseStamped>>& vec_grasping_poses, int ID_grasp,
                            geometry_msgs::PoseStamped& final_pose);

  void trajecto_bari_to_out(geometry_msgs::PoseStamped& final_pose);

  std::vector<std::vector<geometry_msgs::PoseStamped>> load_grasping_pose(float distance_approch, std::string FilePath);

  void full_scenario_grasp_robot(int M, int N);

  void full_scenario_grasp(int M, int N, std::string planni_id);

  geometry_msgs::Quaternion relative_quat_rotation(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2);

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


