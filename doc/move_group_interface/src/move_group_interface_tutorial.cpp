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

//#define M_PI   3.14159265358979323846

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

double Global_time_find_planning = 0;
double Global_time_traj = 0;
double Global_nb_step_traj = 0;
double Global_move_tool = 0;
double NB_fail = 0;
double NUM_TRAJ = 0;


void createArrowMarker(visualization_msgs::Marker& marker, const geometry_msgs::Pose& pose, const Eigen::Vector3d& dir,
                       int id, double scale = 0.1)
{
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.id = id;
  marker.scale.x = 0.1 * scale;
  marker.scale.y = 0.1 * scale;
  marker.scale.z = scale;

  Eigen::Isometry3d pose_eigen;
  tf2::fromMsg(pose, pose_eigen);
  marker.pose = tf2::toMsg(pose_eigen * Eigen::Translation3d(dir * (0.5 * scale)) *
                           Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), dir));

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
}

void createFrameMarkers(visualization_msgs::MarkerArray& markers, const geometry_msgs::PoseStamped& target,
                        const std::string& ns, bool locked = false)
{
  int id = markers.markers.size();
  visualization_msgs::Marker m;
  m.header.frame_id = target.header.frame_id;
  m.ns = ns;
  m.frame_locked = locked;

  createArrowMarker(m, target.pose, Eigen::Vector3d::UnitX(), ++id);
  m.color.r = 1.0;
  markers.markers.push_back(m);
  createArrowMarker(m, target.pose, Eigen::Vector3d::UnitY(), ++id);
  m.color.g = 1.0;
  markers.markers.push_back(m);
  createArrowMarker(m, target.pose, Eigen::Vector3d::UnitZ(), ++id);
  m.color.b = 1.0;
  markers.markers.push_back(m);
}


tf2::Quaternion random_quaternion() {
	//float x, y, z, u, v, w, s;
	//do { x = 2 * rand() - 1; y = 2 * rand() - 1; z = x * x + y * y; } while (z > 1);
	//do { u = 2 * rand() - 1; v = 2 * rand() - 1; w = u * u + v * v; } while (w > 1);
	//s = sqrt((1 - z) / w);
	//PxQuat pxrand = PxQuat(x, y, s * u, s * v);

	float u1, u2, u3;

	u1 = rand() / float(RAND_MAX); //changer division eucli
	u2 = rand() / float(RAND_MAX);
	u3 = rand() / float(RAND_MAX);

	std::cout << u1 << "  " << u2 << "  " << u3 << "  " << std::endl;

	float temp1 = sqrt(u1);
	float temp2 = sqrt(1 - u1);


	//PxQuat pxrand = PxQuat(0.0f, 0.0f, 0.0f, 1.0f);
	//quaternion uniform random
	tf2::Quaternion quatrand = tf2::Quaternion(temp2 * sin(2 * M_PI * u2), temp2 * cos(2 * M_PI * u2), temp1 * sin(2 * M_PI * u3), temp1 * cos(2 * M_PI * u3));

	return(quatrand);
}


void load_bari_in_scene( moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, const Eigen::Vector3d center){


  // TODO CHANGE
  //std::string modelpath = "/home/pc-m/Documents/My-cao/barillet.obj";
  std::string modelpath = "package://geometric_shapes/test/resources/5067976_barillet_005.obj";
  //std::string modelpath = "package://geometric_shapes/test/resources/rc2.obj";
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

        //tf2::Quaternion q_rot;

        // TODO CHANGE
        //q_rot.setRPY(3.14,0.00,0.00); //rk
        //q_rot.setRPY(1.57,3.14,-3.14); //bari
        //q_rot.setRPY(1.60,1.58,-3.14); //bari
        //q_rot.setRPY(1.57,2.20,2.16);  //bari_rot
        //q_rot = random_quaternion();

        // Stuff the new rotation back into the pose. This requires conversion into a msg type
        //tf2::convert(q_rot, pose.orientation);
        pose.orientation.x = -0.5;
        pose.orientation.y = 0.5;
        pose.orientation.z = -0.5;
        pose.orientation.w = 0.5;

        //double res_x = center.x -0.5 + 0.1*xi;
        pose.position.x = -0.08+ 0.05*yi;
        pose.position.y = -0.74+ 0.04*xi;
        pose.position.z = 0.06;

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


void load_bari_in_scene2(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, const Eigen::Vector3d center){

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

  int N_bari_x = 1;
  int N_bari_y = 1;
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

tf2::Quaternion set_quentin(float rx, float ry, float rz){
  tf2::Quaternion q_rotx;
  q_rotx.setRotation(tf2::Vector3(1,0,0),rx);

  tf2::Quaternion q_roty;
  q_roty.setRotation(tf2::Vector3(0,1,0),ry);

  tf2::Quaternion q_rotz;
  q_rotz.setRotation(tf2::Vector3(0,0,1),rz);

  return q_rotx*q_roty*q_rotz;
}

void load_carton_in_scene(moveit_visual_tools::MoveItVisualTools &visual_tools, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, const Eigen::Vector3d center){

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

  tf2::Quaternion q_rot;

  // TODO CHANGE
  //q_rot.setRPY(3.14,0.00,0.00); //rk
  //q_rot.setRPY(1.57,3.14,-3.14); //bari
  q_rot.setRPY(1.57,0.0,0.0); //bari
  //q_rot.setRPY(1.57,2.20,2.16);  //bari_rot
  //q_rot = random_quaternion();

  // Stuff the new rotation back into the pose. This requires conversion into a msg type
  tf2::convert(q_rot, pose.orientation);


  pose.position.x = 0.0;
  pose.position.y = -0.7;
  pose.position.z = 0.2;

  //visual_tools.processCollisionObjectMsg(collision_object_bari, rviz_visual_tools::BROWN);

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


void create_carton_in_scene(moveit_visual_tools::MoveItVisualTools &visual_tools, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, const Eigen::Vector3d center){


  std::vector<double> boxInsideSizeM = {0.269572, 0.184798, 0.177178};
  std::vector<double> boxInsidePoseM = {0.0464935, -0.823521, -0.275351, 337.809, 2.13806, 59.0498};

  geometry_msgs::PoseStamped base_link_moveit_to_halcon;
  base_link_moveit_to_halcon.pose.position.x = 0.0;
  base_link_moveit_to_halcon.pose.position.y = 0.0;
  base_link_moveit_to_halcon.pose.position.z = 0.478;

  Eigen::Isometry3d tf_base_link_moveit_to_halcon;
  tf2::fromMsg(base_link_moveit_to_halcon.pose, tf_base_link_moveit_to_halcon); //pose in bary frame


  geometry_msgs::PoseStamped tf_to_scene;

  tf2::Quaternion q_rot_box;
  q_rot_box = set_quentin(boxInsidePoseM[3]*M_PI/180.0, boxInsidePoseM[4]*M_PI/180.0, boxInsidePoseM[5]*M_PI/180.0);
  tf2::convert(q_rot_box, tf_to_scene.pose.orientation);

  tf_to_scene.pose.position.x = boxInsidePoseM[0];
  tf_to_scene.pose.position.y = boxInsidePoseM[1];
  tf_to_scene.pose.position.z = boxInsidePoseM[2];

  Eigen::Isometry3d tf_to_scene_tot;
  tf2::fromMsg(tf_to_scene.pose, tf_to_scene_tot); //pose in bary frame


  std::vector<double> boxThickSideBottomM = {0.006, 0.016};
  std::cout << "BOX" << boxInsidePoseM.size() << std::endl;



  // Now let's define a collision object ROS message for the robot to avoid.
  moveit_msgs::CollisionObject collision_object_box;
  collision_object_box.header.frame_id = "base_link";
  // The id of the object is used to identify it.
  collision_object_box.id = "carton1";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = boxInsideSizeM[0] + 2* boxThickSideBottomM[0];
  primitive.dimensions[1] = boxInsideSizeM[2];
  primitive.dimensions[2] = boxThickSideBottomM[0];

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose pose;

  tf2::Quaternion q_rot;
  q_rot = set_quentin(-90*M_PI/180.0,0.0,0.0); //bari

  // Stuff the new rotation back into the pose. This requires conversion into a msg type
  tf2::convert(q_rot, pose.orientation);

  pose.position.x = 0.0;
  pose.position.y = -(boxInsideSizeM[1] / 2.0 + boxThickSideBottomM[0] / 2.0);
  pose.position.z = 0.0;

  Eigen::Isometry3d tf_carton;
  tf2::fromMsg(pose, tf_carton); //pose in bary frame

  pose = tf2::toMsg(tf_to_scene_tot * tf_base_link_moveit_to_halcon * tf_carton); //world to bary * pose in bary = pose in world



  collision_object_box.primitives.push_back(primitive);
  collision_object_box.primitive_poses.push_back(pose);
  collision_object_box.operation = collision_object_box.ADD;

  collision_object_baris.push_back(collision_object_box);
//////

  collision_object_box.header.frame_id = "base_link";
  // The id of the object is used to identify it.
  collision_object_box.id = "carton2";
  q_rot = set_quentin(-90*M_PI/180.0,0.0,0.0); //bari

  // Stuff the new rotation back into the pose. This requires conversion into a msg type
  tf2::convert(q_rot, pose.orientation);

  pose.position.x = 0.0;
  pose.position.y = (boxInsideSizeM[1] / 2.0 + boxThickSideBottomM[0] / 2.0);
  pose.position.z = 0.0;

  tf2::fromMsg(pose, tf_carton); //pose in bary frame
  pose = tf2::toMsg(tf_to_scene_tot * tf_base_link_moveit_to_halcon * tf_carton); //world to bary * pose in bary = pose in world


  collision_object_box.primitives.push_back(primitive);
  collision_object_box.primitive_poses.push_back(pose);
  collision_object_box.operation = collision_object_box.ADD;

  collision_object_baris.push_back(collision_object_box);


////////////////////////////////


  collision_object_box.header.frame_id = "base_link";
  // The id of the object is used to identify it.
  collision_object_box.id = "carton3";

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = boxInsideSizeM[2];
  primitive.dimensions[1] = boxInsideSizeM[1];
  primitive.dimensions[2] = boxThickSideBottomM[0];

  q_rot = set_quentin(0.0,-90*M_PI/180.0,0.0); //bari

  // Stuff the new rotation back into the pose. This requires conversion into a msg type
  tf2::convert(q_rot, pose.orientation);

  pose.position.x = (boxInsideSizeM[0] / 2.0 + boxThickSideBottomM[0] / 2.0);
  pose.position.y = 0.0;
  pose.position.z = 0.0;

  tf2::fromMsg(pose, tf_carton); //pose in bary frame
  pose = tf2::toMsg(tf_to_scene_tot * tf_base_link_moveit_to_halcon * tf_carton); //world to bary * pose in bary = pose in world

  collision_object_box.primitives.push_back(primitive);
  collision_object_box.primitive_poses.push_back(pose);
  collision_object_box.operation = collision_object_box.ADD;

  collision_object_baris.push_back(collision_object_box);
//////

  collision_object_box.header.frame_id = "base_link";
  // The id of the object is used to identify it.
  collision_object_box.id = "carton4";
  q_rot = set_quentin(0.0,-90*M_PI/180.0,0.0); //bari

  // Stuff the new rotation back into the pose. This requires conversion into a msg type
  tf2::convert(q_rot, pose.orientation);

  pose.position.x = -(boxInsideSizeM[0] / 2.0 + boxThickSideBottomM[0] / 2.0);
  pose.position.y = 0.0;
  pose.position.z = 0.0;

  tf2::fromMsg(pose, tf_carton); //pose in bary frame
  pose = tf2::toMsg(tf_to_scene_tot * tf_base_link_moveit_to_halcon * tf_carton); //world to bary * pose in bary = pose in world

  collision_object_box.primitives.push_back(primitive);
  collision_object_box.primitive_poses.push_back(pose);
  collision_object_box.operation = collision_object_box.ADD;

  collision_object_baris.push_back(collision_object_box);

/////////////////////////////////////////

  collision_object_box.header.frame_id = "base_link";
  // The id of the object is used to identify it.
  collision_object_box.id = "carton5";

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = boxInsideSizeM[0] + 2* boxThickSideBottomM[0];
  primitive.dimensions[1] = boxInsideSizeM[1] + 2* boxThickSideBottomM[0];
  primitive.dimensions[2] = boxThickSideBottomM[1];

  q_rot = set_quentin(0.0,0.0,0.0); //bari

  // Stuff the new rotation back into the pose. This requires conversion into a msg type
  tf2::convert(q_rot, pose.orientation);

  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = -(boxInsideSizeM[2] + boxThickSideBottomM[1])/ 2.0;

  tf2::fromMsg(pose, tf_carton); //pose in bary frame
  pose = tf2::toMsg(tf_to_scene_tot * tf_base_link_moveit_to_halcon * tf_carton ); //world to bary * pose in bary = pose in world


  collision_object_box.primitives.push_back(primitive);
  collision_object_box.primitive_poses.push_back(pose);
  collision_object_box.operation = collision_object_box.ADD;

  collision_object_baris.push_back(collision_object_box);


  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_object_baris);


}


double distance(double x1, double y1, double z1, double x2, double y2, double z2){

  return sqrt(pow(x1-x2,2)+pow(y1-y2,2)+pow(z1-z2,2));

}

double distance6D(double x1, double y1, double z1, double v1, double u1, double w1, double x2, double y2, double z2, double v2, double u2, double w2){

  return sqrt(pow(x1-x2,2)+pow(y1-y2,2)+pow(z1-z2,2)+pow(v1-v2,2)+pow(u1-u2,2)+pow(w1-w2,2));

}


bool checkIkValidity(robot_state::RobotState* robot_state, const robot_state::JointModelGroup* joint_group, const double* joint_group_variable_values){

  return true;
}


geometry_msgs::Quaternion apply_rotation(geometry_msgs::Quaternion q1 , geometry_msgs::Quaternion q_to_apply){

  geometry_msgs::Quaternion result;
  tf2::Quaternion q_orig, q_rot, q_new;

  // Get the original orientation of 'commanded_pose'
  tf2::convert(q1 , q_orig);
  tf2::convert(q_to_apply , q_rot);


  q_new = q_rot*q_orig;  // Calculate the new orientation
  q_new.normalize();

  // Stuff the new rotation back into the pose. This requires conversion into a msg type
  tf2::convert(q_new, result);
  return result;

}


tf2::Quaternion apply_rotation(tf2::Quaternion q1 , tf2::Quaternion q_rot){

  tf2::Quaternion q_new;


  q_new = q_rot*q1;  // Calculate the new orientation
  q_new.normalize();

  return q_new;

}




 // moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(&isIKStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
   //                           collision_checking_verbose_, only_check_self_collision_, visual_tools_, _1, _2, _3);

std::vector<double> go_to_position_begin(moveit::planning_interface::MoveGroupInterface &move_group_interface , geometry_msgs::PoseStamped target_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ){
  

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



  //std::string planner_id = "SemiPersistentLazyPRMstar";
  std::string planner_id = "SemiPersistentLazyPRMstar";
  move_group_interface.setPlannerId(planner_id);
  move_group_interface.setPlanningTime(3);

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


  moveit_msgs::JointConstraint jc1;
  jc1.joint_name = "joint_1";  
  jc1.position = 0.0;
  jc1.tolerance_above = 0.0; //160 degrés
  jc1.tolerance_below = 2.8;
  jc1.weight = 1.0; 

  moveit_msgs::JointConstraint jc2;
  jc2.joint_name = "joint_2";  
  jc2.position = 0.0;
  jc2.tolerance_above = 1.04; //60
  jc2.tolerance_below = 0.7; //40
  jc2.weight = 1.0; 

  moveit_msgs::JointConstraint jc3;
  jc3.joint_name = "joint_3";
  jc3.position = 0.0;
  jc3.tolerance_above = 2.53; //145
  jc3.tolerance_below = 0.0; //0
  jc3.weight = 1.0;

  moveit_msgs::JointConstraint jc4;
  jc4.joint_name = "joint_4";  
  jc4.position = 0.0;
  jc4.tolerance_above = 2.27; //130
  jc4.tolerance_below = 2.27;
  jc4.weight = 1.0; 


  moveit_msgs::JointConstraint jc5;
  jc5.joint_name = "joint_5";  
  jc5.position = 0.0;
  jc5.tolerance_above = 2.39; //137
  jc5.tolerance_below = 2.0; //115
  jc5.weight = 1.0; 

  moveit_msgs::JointConstraint jc6;
  jc6.joint_name = "joint_6";  
  jc6.position = 0.0;
  jc6.tolerance_above = 3.14; //180
  jc6.tolerance_below = 3.15;
  jc6.weight = 1.0; 

  moveit_msgs::Constraints test_constraints;
  //test_constraints.orientation_constraints.push_back(ocm);
  //test_constraints.position_constraints.push_back(pcm);
  test_constraints.joint_constraints.push_back(jc1);
  test_constraints.joint_constraints.push_back(jc2);
  test_constraints.joint_constraints.push_back(jc3);
  test_constraints.joint_constraints.push_back(jc4);
  test_constraints.joint_constraints.push_back(jc5);
  //test_constraints.joint_constraints.push_back(jc6);
  move_group_interface.setPathConstraints(test_constraints);
  
  // std::string planner_test = move_group_interface.getPlannerId();
  // std::string pipeline_test = move_group_interface.getPlanningPipelineId();
  // std::cout<<"getPlannerId : "<<planner_test<<std::endl;
  // std::cout<<"getPipeline : "<<pipeline_test<<std::endl;

  // //std::cout<<planner_test*<<std::endl;

  // robot_state::RobotState begin_state(*move_group_interface.getCurrentState());

  // std::vector<double> init_state;
  // begin_state.copyJointGroupPositions(joint_model_group, init_state);

  int number_joint = 6;
  // double min_dist = 9000;

  // std::chrono::high_resolution_clock::time_point begin_ik = std::chrono::high_resolution_clock::now();

  // for (int k=0; k<8 ; k++){
  //   robot_state::RobotState final_state(*move_group_interface.getCurrentState());

  //   final_state.setFromIK(joint_model_group,target_pose);


  //   std::vector<double> joint_group_positions_final_ik;
  //   final_state.copyJointGroupPositions(joint_model_group, joint_group_positions_final_ik);



  //   for (int i =0 ; i< number_joint  ; i++ )
  //   {
  //     std::cout << "ik1 i = " << i << " : " << joint_group_positions_final_ik[i]*180/3.14159265 << std::endl;
  //   }

  //   double dist6D = distance6D((double) joint_group_positions_final_ik[0]*180/3.14159265,(double) joint_group_positions_final_ik[1]*180/3.14159265,(double) joint_group_positions_final_ik[2]*180/3.14159265,
  //                            (double) joint_group_positions_final_ik[3]*180/3.14159265,(double) joint_group_positions_final_ik[4]*180/3.14159265,(double) joint_group_positions_final_ik[5]*180/3.14159265,
  //                            (double) init_state[0]*180/3.14159265,                    (double) init_state[1]*180/3.14159265,                    (double) init_state[2]*180/3.14159265,
  //                            (double) init_state[3]*180/3.14159265,                    (double) init_state[4]*180/3.14159265,                    (double) init_state[5]*180/3.14159265);


  //   std::cout << "DISTANCE ULT 6D = " << dist6D << std::endl;

  //   if (dist6D < min_dist){
  //     move_group_interface.setJointValueTarget(final_state);
  //   }

  // }


  // std::chrono::high_resolution_clock::time_point end_ik = std::chrono::high_resolution_clock::now();

  // double time_ik = std::chrono::duration_cast<std::chrono::microseconds>(end_ik-begin_ik).count()/1000000.0;

  // std::cout<< "TIME TO IK TOTAL : " << time_ik << " s " <<std::endl;




  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  //double res_x = center.x -0.5 + 0.1*xi;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.15;

  //move_group_interface.setPoseReferenceFrame("bary0");

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

  for (int i =0 ; i< number_joint  ; i++ )
  {
    std::cout << "angle joint i = " << i << " : " << joint_group_positions[i]*180/3.14159265 << std::endl;
  }
  //std::cout << "test std::cout" << trajecto_state.getWayPoint(0).getVariablePositions << std::endl;  //trajecto_state.getWayPointCount()
  //std::cout << "test std::cout" << trajecto_state.getWayPoint(0) << std::endl;  //trajecto_state.getWayPointCount()

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



  std::string planner_id = "SemiPersistentPRMstar";
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
  //parametre["type"] = "geometric::LazyPRM";
  //std::cout << "parametre[type] : "<< parametre["type"] << std::endl;

  //move_group_interface.setPlannerParams(planner_id, "arm_group", parametre);


  move_group_interface.setPlanningTime(600);
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



  std::string planner_id = "SemiPersistentLazyPRMstar";
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
  //parametre["type"] = "geometric::LazyPRM";
  //std::cout << "parametre[type] : "<< parametre["type"] << std::endl;

  //move_group_interface.setPlannerParams(planner_id, "arm_group", parametre);



  move_group_interface.setPlanningTime(0.5);
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


  moveit_msgs::JointConstraint jc1;
  jc1.joint_name = "joint_1";  
  jc1.position = 0.0;
  jc1.tolerance_above = 0.0; //80 degrés
  jc1.tolerance_below = 2.8;
  jc1.weight = 1.0; 

  moveit_msgs::JointConstraint jc2;
  jc2.joint_name = "joint_2";  
  jc2.position = 0.0;
  jc2.tolerance_above = 1.04; //60
  jc2.tolerance_below = 0.7; //40
  jc2.weight = 1.0; 

  moveit_msgs::JointConstraint jc3;
  jc3.joint_name = "joint_3";
  jc3.position = 0.0;
  jc3.tolerance_above = 2.53; //145
  jc3.tolerance_below = 0.0; //+40
  jc3.weight = 1.0;

  moveit_msgs::JointConstraint jc4;
  jc4.joint_name = "joint_4";  
  jc4.position = 0.0;
  jc4.tolerance_above = 2.27; //130
  jc4.tolerance_below = 2.27;
  jc4.weight = 1.0; 


  moveit_msgs::JointConstraint jc5;
  jc5.joint_name = "joint_5";  
  jc5.position = 0.0;
  jc5.tolerance_above = 2.39; //137
  jc5.tolerance_below = 0; //115
  jc5.weight = 1.0; 

  moveit_msgs::JointConstraint jc6;
  jc6.joint_name = "joint_6";  
  jc6.position = 0.0;
  jc6.tolerance_above = 3.14; //180
  jc6.tolerance_below = 3.15;
  jc6.weight = 1.0; 

  moveit_msgs::Constraints test_constraints;
  //test_constraints.orientation_constraints.push_back(ocm);
  //test_constraints.position_constraints.push_back(pcm);
  test_constraints.joint_constraints.push_back(jc1);
  test_constraints.joint_constraints.push_back(jc2);
  test_constraints.joint_constraints.push_back(jc3);
  test_constraints.joint_constraints.push_back(jc4);
  test_constraints.joint_constraints.push_back(jc5);
  //test_constraints.joint_constraints.push_back(jc6);
  move_group_interface.setPathConstraints(test_constraints);


}



void setup_planner2(moveit::planning_interface::MoveGroupInterface &move_group_interface){

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



  std::string planner_id = "SemiPersistentLazyPRMstar";
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
  //parametre["type"] = "geometric::LazyPRM";
  //std::cout << "parametre[type] : "<< parametre["type"] << std::endl;

  //move_group_interface.setPlannerParams(planner_id, "arm_group", parametre);



  move_group_interface.setPlanningTime(2);
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


void setup_planner_test(moveit::planning_interface::MoveGroupInterface &move_group_interface){

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



  std::string planner_id = "SemiPersistentLazyPRMstar";
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
  //parametre["type"] = "geometric::LazyPRM";
  //std::cout << "parametre[type] : "<< parametre["type"] << std::endl;

  //move_group_interface.setPlannerParams(planner_id, "arm_group", parametre);



  move_group_interface.setPlanningTime(10);
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
  moveit_msgs::JointConstraint jc1;
  jc1.joint_name = "joint_1";  
  jc1.position = 0.0;
  jc1.tolerance_above = 0.0; //80 degrés
  jc1.tolerance_below = 2.8;
  jc1.weight = 1.0; 

  moveit_msgs::JointConstraint jc2;
  jc2.joint_name = "joint_2";  
  jc2.position = 0.0;
  jc2.tolerance_above = 1.04; //60
  jc2.tolerance_below = 0.7; //40
  jc2.weight = 1.0; 

  moveit_msgs::JointConstraint jc3;
  jc3.joint_name = "joint_3";
  jc3.position = 0.0;
  jc3.tolerance_above = 2.53; //145
  jc3.tolerance_below = 0.0; //+40
  jc3.weight = 1.0;

  moveit_msgs::JointConstraint jc4;
  jc4.joint_name = "joint_4";  
  jc4.position = 0.0;
  jc4.tolerance_above = 2.27; //130
  jc4.tolerance_below = 2.27;
  jc4.weight = 1.0; 


  moveit_msgs::JointConstraint jc5;
  jc5.joint_name = "joint_5";  
  jc5.position = 0.0;
  jc5.tolerance_above = 2.39; //137
  jc5.tolerance_below = 0; //115
  jc5.weight = 1.0; 

  moveit_msgs::JointConstraint jc6;
  jc6.joint_name = "joint_6";  
  jc6.position = 0.0;
  jc6.tolerance_above = 3.14; //180
  jc6.tolerance_below = 3.15;
  jc6.weight = 1.0; 

  moveit_msgs::Constraints test_constraints;
  //test_constraints.orientation_constraints.push_back(ocm);
  //test_constraints.position_constraints.push_back(pcm);
  test_constraints.joint_constraints.push_back(jc1);
  test_constraints.joint_constraints.push_back(jc2);
  test_constraints.joint_constraints.push_back(jc3);
  test_constraints.joint_constraints.push_back(jc4);
  test_constraints.joint_constraints.push_back(jc5);
  //test_constraints.joint_constraints.push_back(jc6);
  move_group_interface.setPathConstraints(test_constraints);

}



void setup_planner_test_constrain(moveit::planning_interface::MoveGroupInterface &move_group_interface){

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



  std::string planner_id = "SemiPersistentPRMstar";
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
  //parametre["type"] = "geometric::LazyPRM";
  //std::cout << "parametre[type] : "<< parametre["type"] << std::endl;

  //move_group_interface.setPlannerParams(planner_id, "arm_group", parametre);



  move_group_interface.setPlanningTime(10);
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
  std::cout << "test std::cout" << trajecto_state.getWayPoint(1) << std::endl;  //trajecto_state.getWayPointCount()

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





geometry_msgs::PoseStamped link6_in_bari_grasp( Eigen::Isometry3d tf_tcp_in_bari, float recul){

  geometry_msgs::PoseStamped pose_transformed;

  geometry_msgs::PoseStamped tf_rot_90;

  tf2::Quaternion q_rot90;
  q_rot90.setRPY(-90*M_PI/180.0, 0.0, 0.0);
  tf2::convert(q_rot90, tf_rot_90.pose.orientation);

  tf_rot_90.pose.position.x = 0.0;
  tf_rot_90.pose.position.y = 0.0;
  tf_rot_90.pose.position.z = 0.0;

  Eigen::Isometry3d tfi_rot90;
  tf2::fromMsg(tf_rot_90.pose, tfi_rot90); //pose in bary frame

  //###########


  pose_transformed.pose = tf2::toMsg(tfi_rot90 * tf_tcp_in_bari); //world to bary * pose in bary = pose in world





  geometry_msgs::PoseStamped tf_translation;

  tf2::Quaternion q_test;
  q_test.setRPY( 0.0, 0.0, 0.0);

  tf2::convert(q_test, tf_translation.pose.orientation);

  //double res_x = center.x -0.5 + 0.1*xi;
  tf_translation.pose.position.x = 0.0;
  tf_translation.pose.position.y = 0.0;
  tf_translation.pose.position.z = -recul;
  Eigen::Isometry3d tf_iso_translation;
  tf2::fromMsg(tf_translation.pose, tf_iso_translation); //pose in bary frame



  pose_transformed.pose = tf2::toMsg(tfi_rot90 * tf_tcp_in_bari * tf_iso_translation); //world to bary * pose in bary = pose in world




  geometry_msgs::PoseStamped tf_tool_to_tcp;

  tf2::Quaternion q_test333;
  q_test333 = set_quentin( 20*M_PI/180.0, 0.0, 0.0); //20°

  tf2::convert(q_test333, tf_tool_to_tcp.pose.orientation);

  //double res_x = center.x -0.5 + 0.1*xi;
  tf_tool_to_tcp.pose.position.x = 0.0;
  tf_tool_to_tcp.pose.position.y = -0.040;
  tf_tool_to_tcp.pose.position.z = 0.3735;
  Eigen::Isometry3d tf_iso_tool_to_tcp;
  tf2::fromMsg(tf_tool_to_tcp.pose, tf_iso_tool_to_tcp); //pose in bary frame



  pose_transformed.pose = tf2::toMsg(tfi_rot90 * tf_tcp_in_bari * tf_iso_translation * tf_iso_tool_to_tcp.inverse()); //world to bary * pose in bary = pose in world



  geometry_msgs::PoseStamped tf_link6_to_tool;

  tf2::Quaternion q_test444;
  q_test444 = set_quentin( 0.0, 0.0, -45*M_PI/180.0); //45°

  tf2::convert(q_test444, tf_link6_to_tool.pose.orientation);

  //double res_x = center.x -0.5 + 0.1*xi;
  tf_link6_to_tool.pose.position.x = 0.0;
  tf_link6_to_tool.pose.position.y = 0.0;
  tf_link6_to_tool.pose.position.z = 0.0;
  Eigen::Isometry3d tf_iso_link6_to_tool;
  tf2::fromMsg(tf_link6_to_tool.pose, tf_iso_link6_to_tool); //pose in bary frame



  pose_transformed.pose = tf2::toMsg(tf_tcp_in_bari * tf_iso_translation * tf_iso_tool_to_tcp.inverse() * tf_iso_link6_to_tool.inverse()); //world to bary * pose in bary = pose in world


  return pose_transformed;
}

// translation(m) / Rotation (deg)
// Poses defined in object's frame (x,y,z,rx,ry,rz)
Eigen::Isometry3d create_iso_tcp_in_bari(float tx, float ty, float tz, float rx, float ry, float rz){

  geometry_msgs::PoseStamped tcp_in_bari;
  tf2::Quaternion q_test;
  q_test = set_quentin(rx*M_PI/180.0, ry*M_PI/180.0, rz*M_PI/180.0);
  tf2::convert(q_test, tcp_in_bari.pose.orientation);

  //double res_x = center.x -0.5 + 0.1*xi;
  tcp_in_bari.pose.position.x = 0.0;
  tcp_in_bari.pose.position.y = -0.011;
  tcp_in_bari.pose.position.z = -0.001;
  Eigen::Isometry3d tf_tcp_in_bari;
  tf2::fromMsg(tcp_in_bari.pose, tf_tcp_in_bari); //pose in bary frame

  return tf_tcp_in_bari;

}

std::vector<std::vector<double>> go_to_position(moveit::planning_interface::MoveGroupInterface &move_group_interface ,geometry_msgs::PoseStamped target_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ){


  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  //visual_tools.publishText(text_pose, " current : show target_pose_final", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  //visual_tools.publishAxisLabeled(target_pose, "target_pose_final");
  //visual_tools.trigger(); // to apply changes

  ros::NodeHandle nh;



  // Fetch the current planning scene state once
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  planning_scene_monitor->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor);



  // Visualize frames as rviz markers
  ros::Publisher marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("rviz_visual_tools", 10);
  auto showFrames = [&](geometry_msgs::PoseStamped target, const std::string& eef) {
    visualization_msgs::MarkerArray markers;
    // convert target pose into planning frame
    Eigen::Isometry3d tf;
    tf2::fromMsg(target.pose, tf); //pose in bary frame
    target.pose = tf2::toMsg(planning_scene->getFrameTransform(target.header.frame_id) * tf); //world to bary * pose in bary = pose in world
    target.header.frame_id = planning_scene->getPlanningFrame();
    createFrameMarkers(markers, target, "target");

    // convert eef in pose relative to panda_hand
    target.header.frame_id = "bari0";
    target.pose = tf2::toMsg(planning_scene->getFrameTransform(target.header.frame_id).inverse() *
                             planning_scene->getFrameTransform(eef)); //bari to world * world to eef = bari to eef
    createFrameMarkers(markers, target, "eef", true);

    marker_publisher.publish(markers);
  };



  NUM_TRAJ += 1;



  // robot_state::RobotState begin_state(*move_group_interface.getCurrentState());

  // std::vector<double> init_state;
  // begin_state.copyJointGroupPositions(joint_model_group, init_state);

  int number_joint = 6;

  //std::chrono::high_resolution_clock::time_point begin_ik = std::chrono::high_resolution_clock::now();

  // double min_dist = 9000;


  // for (int k=0; k<8 ; k++){
  //   robot_state::RobotState final_state(*move_group_interface.getCurrentState());

  //   final_state.setFromIK(joint_model_group,target_pose);


  //   std::vector<double> joint_group_positions_final_ik;
  //   final_state.copyJointGroupPositions(joint_model_group, joint_group_positions_final_ik);

  //   planning_scene::PlanningScene planning_scene(kinematic_model);

  //   bool constrained = planning_scene_interface.isStateConstrained(final_state, test_constraints);
  //   ROS_INFO_STREAM("Test 7: final state is "
  //                   << (constrained ? "constrained" : "not constrained"));

  //   for (int i =0 ; i< number_joint  ; i++ )
  //   {
  //     std::cout << "ik1 i = " << i << " : " << joint_group_positions_final_ik[i]*180/3.14159265 << std::endl;
  //   }

  //   double dist6D = distance6D((double) joint_group_positions_final_ik[0]*180/3.14159265,(double) joint_group_positions_final_ik[1]*180/3.14159265,(double) joint_group_positions_final_ik[2]*180/3.14159265,
  //                            (double) joint_group_positions_final_ik[3]*180/3.14159265,(double) joint_group_positions_final_ik[4]*180/3.14159265,(double) joint_group_positions_final_ik[5]*180/3.14159265,
  //                            (double) init_state[0]*180/3.14159265,                    (double) init_state[1]*180/3.14159265,                    (double) init_state[2]*180/3.14159265,
  //                            (double) init_state[3]*180/3.14159265,                    (double) init_state[4]*180/3.14159265,                    (double) init_state[5]*180/3.14159265);


  //   std::cout << "DISTANCE ULT 6D = " << dist6D << std::endl;

  //   if (dist6D < min_dist){
  //     move_group_interface.setJointValueTarget(final_state);
  //   }
  // }


  // std::chrono::high_resolution_clock::time_point end_ik = std::chrono::high_resolution_clock::now();

  // double time_ik = std::chrono::duration_cast<std::chrono::microseconds>(end_ik-begin_ik).count()/1000000.0;

  // std::cout<< "TIME TO IK TOTAL : " << time_ik << " s " <<std::endl;




  // robot_state::RobotState final_state(*move_group_interface.getCurrentState());
  // final_state.setFromIK(joint_model_group,target_pose);

  // move_group_interface.setJointValueTarget(final_state);







  // Eigen::Isometry3d tf_tcp_in_bari;

  // tf_tcp_in_bari = create_iso_tcp_in_bari(0.0, -0.011, -0.001, -90, 0, 90);


  // geometry_msgs::PoseStamped tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, 0.02);
  // tf_transformed.header.frame_id = "bari0";


  showFrames(target_pose, target_pose.header.frame_id);
  visual_tools.publishText(text_pose, "Show target frame", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger(); // to apply changes
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  // std::vector<geometry_msgs::PoseStamped> grasping_poses;

  // Eigen::Isometry3d tf_tcp_in_bari;
  // tf_tcp_in_bari = create_iso_tcp_in_bari(0.0, -0.011, -0.001, -90, 0, 90);
  // geometry_msgs::PoseStamped tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, 0.02);
  // tf_transformed.header.frame_id = "bari0";
  // grasping_poses.push_back(tf_transformed);
  // tf_tcp_in_bari = create_iso_tcp_in_bari(0.0, -0.011, -0.001, -90, 0, -90);
  // tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, 0.02);
  // tf_transformed.header.frame_id = "bari0";
  // grasping_poses.push_back(tf_transformed);



  // move_group_interface.setPoseTargets(grasping_poses);

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
      NB_fail++;
    }

  }

  if (!success) {
    std::cerr << "FAIL TO FIND A PATH AFTER 10 TRY AGAIN" << std::endl;
    exit(0);
  }
  else{

  }
  //my_plan.trajectory_.

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
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



  return trajecto_waypoint_joint;
}


std::vector<std::vector<double>> go_to_position(moveit::planning_interface::MoveGroupInterface &move_group_interface ,std::vector<geometry_msgs::PoseStamped> grasp_poses, int ID_grasp,  moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ){


  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  //visual_tools.publishText(text_pose, " current : show target_pose_final", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  //visual_tools.publishAxisLabeled(target_pose, "target_pose_final");
  //visual_tools.trigger(); // to apply changes

  ros::NodeHandle nh;



  // Fetch the current planning scene state once
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  planning_scene_monitor->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor);



  // Visualize frames as rviz markers
  ros::Publisher marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("rviz_visual_tools", 10);
  auto showFrames = [&](geometry_msgs::PoseStamped target, const std::string& eef) {
    visualization_msgs::MarkerArray markers;
    // convert target pose into planning frame
    Eigen::Isometry3d tf;
    tf2::fromMsg(target.pose, tf); //pose in bary frame
    target.pose = tf2::toMsg(planning_scene->getFrameTransform(target.header.frame_id) * tf); //world to bary * pose in bary = pose in world
    target.header.frame_id = planning_scene->getPlanningFrame();
    createFrameMarkers(markers, target, "target");

    // convert eef in pose relative to panda_hand
    target.header.frame_id = "bari0";
    target.pose = tf2::toMsg(planning_scene->getFrameTransform(target.header.frame_id).inverse() *
                             planning_scene->getFrameTransform(eef)); //bari to world * world to eef = bari to eef
    createFrameMarkers(markers, target, "eef", true);

    marker_publisher.publish(markers);
  };



  NUM_TRAJ += 1;



  // robot_state::RobotState begin_state(*move_group_interface.getCurrentState());

  // std::vector<double> init_state;
  // begin_state.copyJointGroupPositions(joint_model_group, init_state);

  int number_joint = 6;

  //std::chrono::high_resolution_clock::time_point begin_ik = std::chrono::high_resolution_clock::now();

  // double min_dist = 9000;


  // for (int k=0; k<8 ; k++){
  //   robot_state::RobotState final_state(*move_group_interface.getCurrentState());

  //   final_state.setFromIK(joint_model_group,target_pose);


  //   std::vector<double> joint_group_positions_final_ik;
  //   final_state.copyJointGroupPositions(joint_model_group, joint_group_positions_final_ik);

  //   planning_scene::PlanningScene planning_scene(kinematic_model);

  //   bool constrained = planning_scene_interface.isStateConstrained(final_state, test_constraints);
  //   ROS_INFO_STREAM("Test 7: final state is "
  //                   << (constrained ? "constrained" : "not constrained"));

  //   for (int i =0 ; i< number_joint  ; i++ )
  //   {
  //     std::cout << "ik1 i = " << i << " : " << joint_group_positions_final_ik[i]*180/3.14159265 << std::endl;
  //   }

  //   double dist6D = distance6D((double) joint_group_positions_final_ik[0]*180/3.14159265,(double) joint_group_positions_final_ik[1]*180/3.14159265,(double) joint_group_positions_final_ik[2]*180/3.14159265,
  //                            (double) joint_group_positions_final_ik[3]*180/3.14159265,(double) joint_group_positions_final_ik[4]*180/3.14159265,(double) joint_group_positions_final_ik[5]*180/3.14159265,
  //                            (double) init_state[0]*180/3.14159265,                    (double) init_state[1]*180/3.14159265,                    (double) init_state[2]*180/3.14159265,
  //                            (double) init_state[3]*180/3.14159265,                    (double) init_state[4]*180/3.14159265,                    (double) init_state[5]*180/3.14159265);


  //   std::cout << "DISTANCE ULT 6D = " << dist6D << std::endl;

  //   if (dist6D < min_dist){
  //     move_group_interface.setJointValueTarget(final_state);
  //   }
  // }


  // std::chrono::high_resolution_clock::time_point end_ik = std::chrono::high_resolution_clock::now();

  // double time_ik = std::chrono::duration_cast<std::chrono::microseconds>(end_ik-begin_ik).count()/1000000.0;

  // std::cout<< "TIME TO IK TOTAL : " << time_ik << " s " <<std::endl;




  // robot_state::RobotState final_state(*move_group_interface.getCurrentState());
  // final_state.setFromIK(joint_model_group,target_pose);

  // move_group_interface.setJointValueTarget(final_state);







  // Eigen::Isometry3d tf_tcp_in_bari;

  // tf_tcp_in_bari = create_iso_tcp_in_bari(0.0, -0.011, -0.001, -90, 0, 90);


  // geometry_msgs::PoseStamped tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, 0.02);
  // tf_transformed.header.frame_id = "bari0";

  // for (int k=0; k<grasp_poses.size(); k++){
  //   showFrames(grasp_poses[k], grasp_poses[k].header.frame_id);
  //   visual_tools.publishText(text_pose, "Show target frame", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  //   visual_tools.trigger(); // to apply changes
  //   //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  // }
  if (ID_grasp != -1) {



    showFrames(grasp_poses[ID_grasp], grasp_poses[ID_grasp].header.frame_id);
    visual_tools.publishText(text_pose, "Show target frame", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.trigger(); // to apply changes
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


    // std::vector<geometry_msgs::PoseStamped> grasping_poses;

    // Eigen::Isometry3d tf_tcp_in_bari;
    // tf_tcp_in_bari = create_iso_tcp_in_bari(0.0, -0.011, -0.001, -90, 0, 90);
    // geometry_msgs::PoseStamped tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, 0.02);
    // tf_transformed.header.frame_id = "bari0";
    // grasping_poses.push_back(tf_transformed);
    // tf_tcp_in_bari = create_iso_tcp_in_bari(0.0, -0.011, -0.001, -90, 0, -90);
    // tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, 0.02);
    // tf_transformed.header.frame_id = "bari0";
    // grasping_poses.push_back(tf_transformed);



    // move_group_interface.setPoseTargets(grasping_poses);

    move_group_interface.setPoseTarget(grasp_poses[ID_grasp]);


  }
  else {
    move_group_interface.setPoseTargets(grasp_poses);
  }
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
      NB_fail++;
    }

  }

  if (!success) {
    std::cerr << "FAIL TO FIND A PATH AFTER 10 TRY AGAIN" << std::endl;
    exit(0);
  }
  else{

  }
  //my_plan.trajectory_.

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
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



  return trajecto_waypoint_joint;
}


std::vector<std::vector<double>> trajecto_approch(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, const moveit::core::JointModelGroup* joint_model_group, moveit_visual_tools::MoveItVisualTools &visual_tools, geometry_msgs::PoseStamped &pose_goal){

  

  geometry_msgs::Pose start_pose2;



  //double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);
  //ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);



  // Fetch the current planning scene state once
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  planning_scene_monitor->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor);




  Eigen::Isometry3d tf;

  std::vector<geometry_msgs::Pose> vecULT;

  //vecULT.push_back(target_pose3);
  tf2::fromMsg(pose_goal.pose, tf); //pose in bary frame
  start_pose2 = tf2::toMsg(planning_scene->getFrameTransform(pose_goal.header.frame_id) * tf); //world to bary * pose in bary = pose in world

  //visual_tools.publishText(text_pose, " current : show target_pose_final", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  //visual_tools.publishAxisLabeled(start_pose2, "ULTIME2");
  //visual_tools.trigger(); // to apply changes
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");


  vecULT.push_back(start_pose2);

  //moveit_msgs::RobotTrajectory trajectory;

  move_group_interface.setStartStateToCurrentState();

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(vecULT, eef_step, jump_threshold, trajectory, false);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);



  visual_tools.trigger(); // to apply changes
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

  move_group_interface.execute(trajectory);

  visual_tools.trigger(); // to apply changes
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

  robot_trajectory::RobotTrajectory trajecto_state(move_group_interface.getCurrentState()->getRobotModel(), move_group_interface.getName());

  trajecto_state.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory);

  std::vector<std::vector<double>> trajecto_waypoint_joint;
  for (std::size_t i = 1; i < trajecto_state.getWayPointCount(); ++i)
  {

    std::vector<double> waypoint_joint;
    trajecto_state.getWayPointPtr(i)->copyJointGroupPositions(joint_model_group, waypoint_joint);
    trajecto_waypoint_joint.push_back(waypoint_joint);
  }


  return trajecto_waypoint_joint;



}


void trajecto_scan_to_bari(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, geometry_msgs::PoseStamped bari_pose){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  ROS_INFO_NAMED("tutorial", "Go to bari position from scan");
  visual_tools.publishText(text_pose, "Go to bari position from scan", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  std::vector<std::vector<double>> traj6 = go_to_position(move_group_interface, bari_pose, visual_tools, joint_model_group, planning_scene_interface);

}


void trajecto_scan_to_bari(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<std::vector<geometry_msgs::PoseStamped>> bari_poses, int ID_grasp){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  ROS_INFO_NAMED("tutorial", "Go to bari position from scan");
  visual_tools.publishText(text_pose, "Go to bari position from scan", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  std::vector<std::vector<double>> traj6 = go_to_position(move_group_interface, bari_poses[0], ID_grasp, visual_tools, joint_model_group, planning_scene_interface);

}



void trajecto_initial_to_scan_and_bari(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<std::vector<geometry_msgs::PoseStamped>> &bari_poses, int ID_grasp){


  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  move_group_interface.setStartState(*move_group_interface.getCurrentState());


  std::vector<double> traj1 = go_to_position_begin(move_group_interface, scan_pose, visual_tools, joint_model_group, planning_scene_interface);

  visual_tools.publishText(text_pose, "Go to scan position from origin", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  const Eigen::Vector3d center(0.5,0.0,0.0);
  //center.x;

  load_bari_in_scene(planning_scene_interface, move_group_interface, collision_object_baris, center);
  create_carton_in_scene(visual_tools, planning_scene_interface, move_group_interface, collision_object_baris, center);


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
  //visual_tools.publishAxisLabeled(bari_poses[0], "bari_pose");
  visual_tools.trigger(); // to apply changes








  move_group_interface.setStartState(*move_group_interface.getCurrentState());


  trajecto_scan_to_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses, ID_grasp);

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  if (ID_grasp != -1){
    trajecto_approch(move_group_interface, collision_object_baris, planning_scene_interface,joint_model_group, visual_tools, bari_poses[1][ID_grasp]);
  }

}





void trajecto_initial_to_scan_and_bari(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<geometry_msgs::PoseStamped> &bari_poses){


  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  move_group_interface.setStartState(*move_group_interface.getCurrentState());


  std::vector<double> traj1 = go_to_position_begin(move_group_interface, scan_pose, visual_tools, joint_model_group, planning_scene_interface);

  visual_tools.publishText(text_pose, "Go to scan position from origin", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  const Eigen::Vector3d center(0.5,0.0,0.0);
  //center.x;

  load_bari_in_scene(planning_scene_interface, move_group_interface, collision_object_baris, center);
  create_carton_in_scene(visual_tools, planning_scene_interface, move_group_interface, collision_object_baris, center);


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
  //visual_tools.publishAxisLabeled(bari_poses[0], "bari_pose");
  visual_tools.trigger(); // to apply changes








  move_group_interface.setStartState(*move_group_interface.getCurrentState());


  trajecto_scan_to_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses[0]);

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  //trajecto_approch(move_group_interface, collision_object_baris, planning_scene_interface, visual_tools, bari_poses[0]);


}




void trajecto_initial_to_scan_and_bari2(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<geometry_msgs::PoseStamped> &bari_poses){


  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  move_group_interface.setStartState(*move_group_interface.getCurrentState());


  std::vector<double> traj1 = go_to_position_begin(move_group_interface, scan_pose, visual_tools, joint_model_group, planning_scene_interface);

  visual_tools.publishText(text_pose, "Go to scan position from origin", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  const Eigen::Vector3d center(0.5,0.0,0.0);
  //center.x;

  load_bari_in_scene2(planning_scene_interface, move_group_interface, collision_object_baris, center);


  std::chrono::seconds dura70(70);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  //ROS_INFO_NAMED("tutorial", "Add an object into the world");
  //planning_scene_interface.addCollisionObjects(collision_objects);


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");




  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, " current : show bari position", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.publishAxisLabeled(bari_poses[0].pose, "bari_pose");
  visual_tools.trigger(); // to apply changes






}



std::vector<std::vector<double>> trajecto_bari_to_scan(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  ROS_INFO_NAMED("tutorial", "Go to scan position from bari");
  visual_tools.publishText(text_pose, "Go to scan position from bari", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  std::vector<std::vector<double>> traj6 = go_to_position(move_group_interface, scan_pose, visual_tools, joint_model_group, planning_scene_interface);

  return traj6;

}


std::vector<std::vector<double>> trajecto_scan_to_out(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, geometry_msgs::PoseStamped &final_pose){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  ROS_INFO_NAMED("tutorial", "Go to out position from scan");
  visual_tools.publishText(text_pose, "Go to out position from scan", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  std::vector<std::vector<double>> traj6 = go_to_position(move_group_interface, final_pose, visual_tools, joint_model_group, planning_scene_interface);

  return traj6;

}



std::vector<std::vector<double>> trajecto_bari_to_out_by_scan(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, geometry_msgs::PoseStamped &final_pose, std::vector<std::vector<geometry_msgs::PoseStamped>> &vec_grasping_poses, int ID_grasp){

  std::vector<std::vector<double>> traj0;
  std::vector<std::vector<double>> traj1;
  std::vector<std::vector<double>> traj2;

  if (ID_grasp != -1){
    traj0 = trajecto_approch(move_group_interface, collision_object_baris, planning_scene_interface,joint_model_group, visual_tools, vec_grasping_poses[0][ID_grasp]);
  }

  std::cout<<"size traj"<<traj0.size()<<std::endl;

  traj1 = trajecto_bari_to_scan(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group);

  traj0.insert(traj0.end(),std::make_move_iterator(traj1.begin()),std::make_move_iterator(traj1.end()));

  std::cout<<"size traj"<<traj0.size()<<std::endl;


  traj2 = trajecto_scan_to_out(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, final_pose);

  traj0.insert(traj0.end(),std::make_move_iterator(traj2.begin()),std::make_move_iterator(traj2.end()));

  std::cout<<"size traj"<<traj0.size()<<std::endl;

  ROS_INFO_NAMED("tutorial", "Go to bari position from out"); 
  //visual_tools.publishText(text_pose, "Go to bari position from out", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  //visual_tools.trigger();



  std::ofstream myfile ("/home/guillaume/ws_moveit/example.txt");
  if (myfile.is_open())
  {
    for (int k=0; k<traj0.size(); k++){
      myfile << "f " << traj0[k][0]*180.0/M_PI << " " << traj0[k][1]*180.0/M_PI  << " " << traj0[k][2]*180.0/M_PI  << " " << traj0[k][3]*180.0/M_PI  << " " << traj0[k][4]*180.0/M_PI  << " " << traj0[k][5]*180.0/M_PI  << "\n";
    }
    myfile.close();
  }
  else std::cout << "Unable to open file";

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  return traj0;

}

void trajecto_bari_to_out_by_scan(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, geometry_msgs::PoseStamped &final_pose ){


  trajecto_bari_to_scan(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group);

  trajecto_scan_to_out(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, final_pose);

}

void trajecto_out_to_bari(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, geometry_msgs::PoseStamped &bari_pose){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  ROS_INFO_NAMED("tutorial", "Go to bari position from out"); 
  visual_tools.publishText(text_pose, "Go to bari position from out", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  std::vector<std::vector<double>> traj5 = go_to_position(move_group_interface, bari_pose, visual_tools, joint_model_group, planning_scene_interface);


}

void trajecto_out_to_bari(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<std::vector<geometry_msgs::PoseStamped>> &vec_grasping_poses, int ID_grasp){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  ROS_INFO_NAMED("tutorial", "Go to bari position from out"); 
  visual_tools.publishText(text_pose, "Go to bari position from out", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  std::vector<std::vector<double>> traj5 = go_to_position(move_group_interface, vec_grasping_poses[0], ID_grasp, visual_tools, joint_model_group, planning_scene_interface);

  if (ID_grasp != -1){
    trajecto_approch(move_group_interface, collision_object_baris, planning_scene_interface, joint_model_group, visual_tools, vec_grasping_poses[1][ID_grasp]);
  }


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");


  // geometry_msgs::Pose start_pose2;



  // //double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);
  // //ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);



  // // Fetch the current planning scene state once
  // auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  // planning_scene_monitor->requestPlanningSceneState();
  // planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor);




  // Eigen::Isometry3d tf;

  // std::vector<geometry_msgs::Pose> vecULT;

  // //vecULT.push_back(target_pose3);
  // tf2::fromMsg(bari_pose_true.pose, tf); //pose in bary frame
  // start_pose2 = tf2::toMsg(planning_scene->getFrameTransform(bari_pose_before.header.frame_id) * tf); //world to bary * pose in bary = pose in world

  // visual_tools.publishText(text_pose, " current : show target_pose_final", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  // visual_tools.publishAxisLabeled(start_pose2, "ULTIME2");
  // visual_tools.trigger(); // to apply changes
  // //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");


  // vecULT.push_back(start_pose2);

  // //moveit_msgs::RobotTrajectory trajectory;

  // move_group_interface.setStartStateToCurrentState();

  // moveit_msgs::RobotTrajectory trajectory;
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.01;
  // double fraction = move_group_interface.computeCartesianPath(vecULT, eef_step, jump_threshold, trajectory, false);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);



  // visual_tools.trigger(); // to apply changes
  // //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

  // move_group_interface.execute(trajectory);

  // visual_tools.trigger(); // to apply changes
  // //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");



}



void trajecto_bari_to_out(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group,std::vector<std::vector<geometry_msgs::PoseStamped>> &vec_grasping_poses, int ID_grasp, geometry_msgs::PoseStamped &final_pose){


  if (ID_grasp != -1){
    trajecto_approch(move_group_interface, collision_object_baris, planning_scene_interface, joint_model_group, visual_tools, vec_grasping_poses[0][ID_grasp]);
  }
  move_group_interface.setStartStateToCurrentState();

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  std::vector<std::vector<double>> traj6 = go_to_position(move_group_interface, final_pose, visual_tools, joint_model_group ,planning_scene_interface);

}


void trajecto_bari_to_out(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, geometry_msgs::PoseStamped &final_pose){



  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  ROS_INFO_NAMED("tutorial", "Go to out position from bari"); 
  visual_tools.publishText(text_pose, "Go to out position from bari", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  std::vector<std::vector<double>> traj6 = go_to_position(move_group_interface, final_pose, visual_tools, joint_model_group ,planning_scene_interface);

}



void full_scenario(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<geometry_msgs::PoseStamped> &final_poses, std::vector<geometry_msgs::PoseStamped> &bari_poses,std::vector<geometry_msgs::PoseStamped> &grasping_poses, int M, int N){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  std::chrono::seconds dura70(70);
  
  trajecto_initial_to_scan_and_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses);

  setup_planner(move_group_interface);


  for (int i = 0; i < M; i++){

    // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
    // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
    ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1));
    move_group_interface.attachObject(collision_object_baris[i*(N+1)].id, "link_tool");

    //moveit_msgs::AttachedCollisionObject aco;
    //aco.object.id = collision_object_baris[0].id;
    //aco.link_name = "link_tool";
    //planning_scene_interface.applyAttachedCollisionObject(aco);


    //std::this_thread::sleep_for(dura70);


    // std::map<std::string, moveit_msgs::CollisionObject> tab_objects = planning_scene_interface.getObjects();
    // std::map<std::string, moveit_msgs::AttachedCollisionObject> tab_attached = planning_scene_interface.getAttachedObjects();


    // std::cout << "###########CollisionObject#########" << std::endl;

    // std::map<std::string, moveit_msgs::CollisionObject>::iterator it_object;
    // int compteur = 0;
    // for (it_object = tab_objects.begin(); it_object != tab_objects.end(); it_object++)
    // //for (it_object = tab_objects.begin(); it_object != 20; it_object++)
    // {
    //   compteur++;
    //   if (compteur >= 20){
    //     break;
    //   }
    //   std::cout << it_object->first    // string (key)
    //   << " : "
    //   << it_object->second   // string's value 
    //   << std::endl;
    // }

    // std::cout << "##########AttachedObject###########" << std::endl;

    // std::map<std::string, moveit_msgs::AttachedCollisionObject>::iterator it_attached;
    // int compteur2 = 0;
    // for (it_attached = tab_attached.begin(); it_attached != tab_attached.end(); it_attached++)
    // //for (it_attached = tab_attached.begin(); it_attached != 20; it_attached++)
    // {
    //   compteur2++;
    //   if (compteur2 >= 20){
    //     break;
    //   }
    //   std::cout << it_attached->first    // string (key)
    //   << " : "
    //   << it_attached->second   // string's value 
    //   << std::endl;
    // }


    visual_tools.publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.trigger();

    move_group_interface.setStartStateToCurrentState();

    // moveit::core::RobotStatePtr start_state;
    // start_state = move_group_interface.getCurrentState();

    // // moveit_msgs::RobotState rob_st;
    // // bool copy_attached_bodies = true;

    // // robot_state::RobotState rs = boost::get<robot_state::RobotState>(start_state);
    // // moveit::core::robotStateToRobotStateMsg(rs, rob_st, copy_attached_bodies);


    // moveit_msgs::RobotState rob_st;
    // bool copy_attached_bodies = true;
    // //robot_state::RobotState rs = boost::get<robot_state::RobotState>(start_state);
    // robot_state::robotStateToRobotStateMsg(*start_state, rob_st, copy_attached_bodies);


    // //move_group_interface.setStartState(*start_state);
    // move_group_interface.setStartState(rob_st);
    
    //move_group_interface.setStartState(*move_group_interface.getCurrentState());
    
    // //planning_scene_interface.applyCollisionObject()

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

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


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





      //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");





      // Replan, but now with the object in hand.
      move_group_interface.setStartStateToCurrentState();

      trajecto_bari_to_out(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group,final_poses[i*(N+1) + j + 1]);


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





      //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");






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

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

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


void full_scenario_grasp(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<geometry_msgs::PoseStamped> &final_poses, std::vector<geometry_msgs::PoseStamped> &bari_poses,std::vector<std::vector<geometry_msgs::PoseStamped>> &vec_grasping_poses, int M, int N){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  std::chrono::seconds dura70(70);

  //TODO CHANGE
  int ID_grasp = 13; //bari
  //int ID_grasp = 1;

  if (ID_grasp != -1){
    vec_grasping_poses[0][ID_grasp].header.frame_id = "bari0";
    vec_grasping_poses[1][ID_grasp].header.frame_id = "bari0";
  }
  else{
    for (int k=0; k<vec_grasping_poses[0].size(); k++){
      vec_grasping_poses[0][k].header.frame_id = "bari0";
      vec_grasping_poses[1][k].header.frame_id = "bari0";
    }
  }

  trajecto_initial_to_scan_and_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, vec_grasping_poses, ID_grasp);

  setup_planner(move_group_interface);


  for (int i = 0; i < M; i++){
    

    std::cout<< "id bari : " << collision_object_baris[i*(N+1)].id << std::endl;

    // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
    // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
    ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1));
    move_group_interface.attachObject(collision_object_baris[i*(N+1)].id, "link_tool");

    //moveit_msgs::AttachedCollisionObject aco;
    //aco.object.id = collision_object_baris[0].id;
    //aco.link_name = "link_tool";
    //planning_scene_interface.applyAttachedCollisionObject(aco);


    //std::this_thread::sleep_for(dura70);


    // std::map<std::string, moveit_msgs::CollisionObject> tab_objects = planning_scene_interface.getObjects();
    // std::map<std::string, moveit_msgs::AttachedCollisionObject> tab_attached = planning_scene_interface.getAttachedObjects();


    // std::cout << "###########CollisionObject#########" << std::endl;

    // std::map<std::string, moveit_msgs::CollisionObject>::iterator it_object;
    // int compteur = 0;
    // for (it_object = tab_objects.begin(); it_object != tab_objects.end(); it_object++)
    // //for (it_object = tab_objects.begin(); it_object != 20; it_object++)
    // {
    //   compteur++;
    //   if (compteur >= 20){
    //     break;
    //   }
    //   std::cout << it_object->first    // string (key)
    //   << " : "
    //   << it_object->second   // string's value 
    //   << std::endl;
    // }

    // std::cout << "##########AttachedObject###########" << std::endl;

    // std::map<std::string, moveit_msgs::AttachedCollisionObject>::iterator it_attached;
    // int compteur2 = 0;
    // for (it_attached = tab_attached.begin(); it_attached != tab_attached.end(); it_attached++)
    // //for (it_attached = tab_attached.begin(); it_attached != 20; it_attached++)
    // {
    //   compteur2++;
    //   if (compteur2 >= 20){
    //     break;
    //   }
    //   std::cout << it_attached->first    // string (key)
    //   << " : "
    //   << it_attached->second   // string's value 
    //   << std::endl;
    // }


    visual_tools.publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.trigger();

    move_group_interface.setStartStateToCurrentState();

    // moveit::core::RobotStatePtr start_state;
    // start_state = move_group_interface.getCurrentState();

    // // moveit_msgs::RobotState rob_st;
    // // bool copy_attached_bodies = true;

    // // robot_state::RobotState rs = boost::get<robot_state::RobotState>(start_state);
    // // moveit::core::robotStateToRobotStateMsg(rs, rob_st, copy_attached_bodies);


    // moveit_msgs::RobotState rob_st;
    // bool copy_attached_bodies = true;
    // //robot_state::RobotState rs = boost::get<robot_state::RobotState>(start_state);
    // robot_state::robotStateToRobotStateMsg(*start_state, rob_st, copy_attached_bodies);


    // //move_group_interface.setStartState(*start_state);
    // move_group_interface.setStartState(rob_st);
    
    //move_group_interface.setStartState(*move_group_interface.getCurrentState());
    
    // //planning_scene_interface.applyCollisionObject()

    trajecto_bari_to_out_by_scan(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, final_poses[i*(N+1)], vec_grasping_poses, ID_grasp);


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

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


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


      std::cout<< "id bari : " << collision_object_baris[i*(N+1) + j + 1].id << std::endl;
      if (ID_grasp != -1){
        vec_grasping_poses[0][ID_grasp].header.frame_id = collision_object_baris[i*(N+1) + j + 1].id;
        vec_grasping_poses[1][ID_grasp].header.frame_id = collision_object_baris[i*(N+1) + j + 1].id;
      }
      else{
        for (int k=0; k<vec_grasping_poses[0].size(); k++){
          vec_grasping_poses[0][k].header.frame_id = collision_object_baris[i*(N+1) + j + 1].id;
          vec_grasping_poses[1][k].header.frame_id = collision_object_baris[i*(N+1) + j + 1].id;
        }
      }

      trajecto_out_to_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, vec_grasping_poses, ID_grasp);


      // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
      // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
      ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1) + j + 1);

      move_group_interface.attachObject(collision_object_baris[i*(N+1) + j + 1].id, "link_tool");

      visual_tools.publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      visual_tools.trigger();

      //std::chrono::seconds dura10(10);


      move_group_interface.setStartStateToCurrentState();





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





      //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");





      // Replan, but now with the object in hand.
      move_group_interface.setStartStateToCurrentState();

      trajecto_bari_to_out(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group,vec_grasping_poses, ID_grasp, final_poses[i*(N+1) + j + 1]);


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





      //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");






      // Show text in RViz of status
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "Object detached from robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      visual_tools.trigger();


      /* Wait for MoveGroup to receive and process the attached collision object message */
      //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");



    }



    move_group_interface.setStartState(*move_group_interface.getCurrentState());


    std::cout<< "id bari : " << collision_object_baris[i*(N+1) + N + 1].id << std::endl;

    if (ID_grasp != -1){
      vec_grasping_poses[0][ID_grasp].header.frame_id = collision_object_baris[i*(N+1) + N + 1].id;
      vec_grasping_poses[1][ID_grasp].header.frame_id = collision_object_baris[i*(N+1) + N + 1].id;
    }
    else{
      for (int k=0; k<vec_grasping_poses[0].size(); k++){
        vec_grasping_poses[0][k].header.frame_id = collision_object_baris[i*(N+1) + N + 1].id;
        vec_grasping_poses[1][k].header.frame_id = collision_object_baris[i*(N+1) + N + 1].id;
      }
    }


    trajecto_out_to_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, vec_grasping_poses, ID_grasp);

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




    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  }



  // Replan, but now with the object in hand.
  move_group_interface.setStartStateToCurrentState();


  trajecto_bari_to_out(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group,vec_grasping_poses, ID_grasp, final_poses[(M-1)*(N+1) + N + 1]);


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

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

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

  /* Wait for MoveGroup to receive and process the attached collision object message */
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

  //object_ids.push_back(object_to_attach.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Objects removed", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();


}




void full_scenario_without_attach(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<geometry_msgs::PoseStamped> &final_poses, std::vector<geometry_msgs::PoseStamped> &bari_poses, int M, int N){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  std::chrono::seconds dura70(70);
  
  trajecto_initial_to_scan_and_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses);

  setup_planner(move_group_interface);


  for (int i = 0; i < M; i++){

    // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
    // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
    //ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1));
    //move_group_interface.attachObject(collision_object_baris[i*(N+1)].id, "link_tool");

    //std::this_thread::sleep_for(dura70);


    // std::map<std::string, moveit_msgs::CollisionObject> tab_objects = planning_scene_interface.getObjects();
    // std::map<std::string, moveit_msgs::AttachedCollisionObject> tab_attached = planning_scene_interface.getAttachedObjects();


    // std::cout << "###########CollisionObject#########" << std::endl;

    // std::map<std::string, moveit_msgs::CollisionObject>::iterator it_object;
    // int compteur = 0;
    // for (it_object = tab_objects.begin(); it_object != tab_objects.end(); it_object++)
    // //for (it_object = tab_objects.begin(); it_object != 20; it_object++)
    // {
    //   compteur++;
    //   if (compteur >= 20){
    //     break;
    //   }
    //   std::cout << it_object->first    // string (key)
    //   << " : "
    //   << it_object->second   // string's value 
    //   << std::endl;
    // }

    // std::cout << "##########AttachedObject###########" << std::endl;

    // std::map<std::string, moveit_msgs::AttachedCollisionObject>::iterator it_attached;
    // int compteur2 = 0;
    // for (it_attached = tab_attached.begin(); it_attached != tab_attached.end(); it_attached++)
    // //for (it_attached = tab_attached.begin(); it_attached != 20; it_attached++)
    // {
    //   compteur2++;
    //   if (compteur2 >= 20){
    //     break;
    //   }
    //   std::cout << it_attached->first    // string (key)
    //   << " : "
    //   << it_attached->second   // string's value 
    //   << std::endl;
    // }


    //visual_tools.publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    //visual_tools.trigger();

    move_group_interface.setStartStateToCurrentState();

    // moveit::core::RobotStatePtr start_state;
    // start_state = move_group_interface.getStartState();

    // move_group_interface.setStartState(*start_state);
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
    //ROS_INFO_NAMED("tutorial", "Detach the object from the robot : %d", i*(N+1));
    //move_group_interface.detachObject(collision_object_baris[i*(N+1)].id);

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


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
      //ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1) + j + 1);

      //move_group_interface.attachObject(collision_object_baris[i*(N+1) + j + 1].id, "link_tool");

      //visual_tools.publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      //visual_tools.trigger();

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
      //ROS_INFO_NAMED("tutorial", "Detach the object from the robot : %d", i*(N+1) + j + 1);
      //move_group_interface.detachObject(collision_object_baris[i*(N+1) + j + 1].id);

      //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

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
    //ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1) + N + 1);

    //move_group_interface.attachObject(collision_object_baris[i*(N+1) + N + 1].id, "link_tool");

    //visual_tools.publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    //visual_tools.trigger();

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
  //ROS_INFO_NAMED("tutorial", "Detach the object from the robot : %d", (M-1)*(N+1) + N + 1);
  //move_group_interface.detachObject(collision_object_baris[(M-1)*(N+1) + N + 1].id);

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Show text in RViz of status
  //visual_tools.deleteAllMarkers();
  //visual_tools.publishText(text_pose, "Object detached from robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
 // visual_tools.trigger();


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


void scenario_test_out_attach(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<geometry_msgs::PoseStamped> &final_poses, std::vector<geometry_msgs::PoseStamped> &bari_poses, int M, int N){

  //#####
  //##### begin scenario #####
  //#####

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  
  trajecto_initial_to_scan_and_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses);


  setup_planner_test(move_group_interface);
  //setup_planner_test_constrain(move_group_interface);

  // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
  // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", 0);

  std::chrono::seconds dura70(2);
  for (int i = 0; i < 10; i++){


    // moveit_msgs::AttachedCollisionObject aco;
    // aco.object.id = collision_object_baris[0].id;
    // aco.link_name = "link_tool";
    // planning_scene_interface.applyAttachedCollisionObject(aco);


    move_group_interface.attachObject(collision_object_baris[0].id, "link_tool");


    move_group_interface.setStartStateToCurrentState();

    trajecto_bari_to_out(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, final_poses[0]);

    move_group_interface.setStartStateToCurrentState();

    trajecto_out_to_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses[0]);

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
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot : %d", 0);
  move_group_interface.detachObject(collision_object_baris[0].id);

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



void scenario_test_scan_attach(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<geometry_msgs::PoseStamped> &final_poses, std::vector<geometry_msgs::PoseStamped> &bari_poses, int M, int N){

  //#####
  //##### begin scenario #####
  //#####

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  
  trajecto_initial_to_scan_and_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses);


  setup_planner_test(move_group_interface);
  //setup_planner_test_constrain(move_group_interface);

  // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
  // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", 0);

  move_group_interface.attachObject(collision_object_baris[0].id, "link_tool");


  std::chrono::seconds dura70(2);
  for (int i = 0; i < 10; i++){


    //moveit_msgs::AttachedCollisionObject aco;
    //aco.object.id = collision_object_baris[0].id;
    //aco.link_name = "link_tool";
    //planning_scene_interface.applyAttachedCollisionObject(aco);


    move_group_interface.setStartStateToCurrentState();

    trajecto_bari_to_scan(move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group);

    move_group_interface.setStartStateToCurrentState();

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
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot : %d", 0);
  move_group_interface.detachObject(collision_object_baris[0].id);

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

geometry_msgs::Quaternion relative_quat_rotation(geometry_msgs::Quaternion q1 , geometry_msgs::Quaternion q2){

  geometry_msgs::Quaternion result;
  tf2::Quaternion q_orig1, q_inv1, q_orig2, q_new;

  tf2::convert(q1, q_orig1);
  tf2::convert(q2, q_orig2);

  q_inv1 = q_orig1;
  q_inv1[3] = - q_inv1[3]; //inverse quat

  q_new = q_orig2*q_inv1;
  q_new.normalize();

  tf2::convert(q_new, result);
  return result;
}




void pick(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, moveit_visual_tools::MoveItVisualTools &visual_tools)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  std::vector<std::string> object_id;
  object_id.push_back(collision_object_baris[0].id);



  std::map<std::string, geometry_msgs::Pose> map_pose = planning_scene_interface.getObjectPoses(object_id);
  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
  // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
  // transform from `"panda_link8"` to the palm of the end effector.
  grasps[0].grasp_pose.header.frame_id = "base_link";
  // tf2::Quaternion orientation;
  //orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
  grasps[0].grasp_pose.pose = map_pose[collision_object_baris[0].id];

  std::cout<< grasps[0].grasp_pose.pose <<std::endl;
  visual_tools.publishAxisLabeled(grasps[0].grasp_pose.pose, "bari_pose");
  visual_tools.trigger(); // to apply changes


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  geometry_msgs::PoseStamped base_pose;
  base_pose.pose.orientation.w=1.0;

  geometry_msgs::PoseStamped tf_pose;
  tf_pose.pose = grasps[0].grasp_pose.pose;
  tf_pose.header.frame_id = grasps[0].grasp_pose.header.frame_id;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);


  geometry_msgs::PointStamped pointstamp ;
  pointstamp.header.frame_id = "link_finger1";
  pointstamp.header.stamp = ros::Time(0);
  pointstamp.point.x = 1.0;
  pointstamp.point.y = 0.0;
  pointstamp.point.z = 0.0;

  // tf2_listener.transformPoint("base_link", pointstamp );

  // visual_tools.publishAxisLabeled(pointstamp, "tcp_in_link");
  // visual_tools.trigger(); // to apply changes


  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");





  geometry_msgs::TransformStamped tcp_in_link_finger1; // My frames are named "base_link" and "leap_motion"
  tcp_in_link_finger1.header.frame_id = "link_finger1";
  tcp_in_link_finger1.child_frame_id = "link_TCP";
  tcp_in_link_finger1.header.stamp = ros::Time::now();
  tcp_in_link_finger1.transform.translation.x = 0.0;
  tcp_in_link_finger1.transform.translation.y = 0.0;
  tcp_in_link_finger1.transform.translation.z = 1.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  tcp_in_link_finger1.transform.rotation.x = q.x();
  tcp_in_link_finger1.transform.rotation.y = q.y();
  tcp_in_link_finger1.transform.rotation.z = q.z();
  tcp_in_link_finger1.transform.rotation.w = q.w();

  geometry_msgs::PoseStamped pose_finger = move_group.getCurrentPose("link_finger1") ;





  // tf::Transform transform;

  // transform.setOrigin( tf::Vector3(0.0, 0.5, 0.0) );
  // transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
  // tcp_in_link_finger1 = geometry_msgs::TransformStamped(transform, ros::Time::now(), "link_finger1", "link_finger1");

  tf2::doTransform(pose_finger.pose, grasps[0].grasp_pose.pose, tcp_in_link_finger1); // robot_pose is the PoseStamped I want to transform


  std::cout<< grasps[0].grasp_pose.pose <<std::endl;

  visual_tools.publishAxisLabeled(grasps[0].grasp_pose.pose, "tcp_in_link");
  visual_tools.trigger(); // to apply changes


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  geometry_msgs::PoseStamped pose_link6 = move_group.getCurrentPose("link_6") ;


  geometry_msgs::PoseStamped pose_link_tool = move_group.getCurrentPose("link_tool") ;


  geometry_msgs::TransformStamped base_link_to_leap_motion; // My frames are named "base_link" and "leap_motion"

  base_link_to_leap_motion = tf_buffer.lookupTransform( "link_6", "link_finger1", ros::Time(0), ros::Duration(50.0) );

  std::cout<< "pose_link6 " << pose_link6 <<std::endl;
  std::cout<< "pose_finger " << pose_finger <<std::endl;

  geometry_msgs::PoseStamped manual_tf_to_apply;


  manual_tf_to_apply.pose.position.x = pose_finger.pose.position.x - pose_link6.pose.position.x;
  manual_tf_to_apply.pose.position.y = pose_finger.pose.position.y - pose_link6.pose.position.y;
  manual_tf_to_apply.pose.position.z = pose_finger.pose.position.z - pose_link6.pose.position.z;


  manual_tf_to_apply.pose.orientation = relative_quat_rotation(pose_link6.pose.orientation ,pose_finger.pose.orientation );

  std::cout<< "relative tf pose " << manual_tf_to_apply <<std::endl;


  geometry_msgs::PoseStamped manual_goal;

  manual_goal = base_pose;

  manual_goal.pose.position.x = manual_goal.pose.position.x + manual_tf_to_apply.pose.position.x;
  manual_goal.pose.position.y = manual_goal.pose.position.y + manual_tf_to_apply.pose.position.y;
  manual_goal.pose.position.z = manual_goal.pose.position.z + manual_tf_to_apply.pose.position.z;

  manual_goal.pose.orientation = apply_rotation(manual_goal.pose.orientation, manual_tf_to_apply.pose.orientation);

  std::cout<< "manual_goal " << manual_goal <<std::endl;


  visual_tools.publishAxisLabeled(manual_goal.pose, "TEST RESULT QUAT");
  visual_tools.trigger(); // to apply changes


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");




  std::cout<< "manual_goal " << manual_goal <<std::endl;


  std::cout<< "tf_link_tool_link_6 ? : " << base_link_to_leap_motion <<std::endl;

  std::cout<< "grasps[0].grasp_pose.pose before  ? : " << grasps[0].grasp_pose.pose <<std::endl;
  std::cout<< "base_pose  ? : " << base_pose <<std::endl;

  grasps[0].grasp_pose.pose = base_pose.pose;

  tf2::doTransform(base_pose, grasps[0].grasp_pose, base_link_to_leap_motion); // robot_pose is the PoseStamped I want to transform

  std::cout<< "link_finger1_with_tf_final ? : " << grasps[0].grasp_pose.pose <<std::endl;

  //::cout<< "link_6 ? : " << pose_link6.pose <<std::endl;


  std::cout<< grasps[0].grasp_pose.pose <<std::endl;

  visual_tools.publishAxisLabeled(pose_link6.pose, "link6");
  visual_tools.trigger(); // to apply changes

  visual_tools.publishAxisLabeled(grasps[0].grasp_pose.pose, "link_tool_in_link6");
  visual_tools.trigger(); // to apply changes


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  geometry_msgs::TransformStamped base_link_to_leap_motion2; // My frames are named "base_link" and "leap_motion"

  base_link_to_leap_motion2 = tf_buffer.lookupTransform( "link_6", "link_tool", ros::Time(0), ros::Duration(1.0) );


  tf2::doTransform(map_pose[collision_object_baris[0].id], grasps[0].grasp_pose.pose, base_link_to_leap_motion2); // robot_pose is the PoseStamped I want to transform


  std::cout<< grasps[0].grasp_pose.pose <<std::endl;

  visual_tools.publishAxisLabeled(grasps[0].grasp_pose.pose, "bari_pose_tf2");
  visual_tools.trigger(); // to apply changes


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  tf2::doTransform(map_pose[collision_object_baris[0].id], grasps[0].grasp_pose.pose, base_link_to_leap_motion2); // robot_pose is the PoseStamped I want to transform


  std::cout<< grasps[0].grasp_pose.pose <<std::endl;

  visual_tools.publishAxisLabeled(grasps[0].grasp_pose.pose, "bari_pose_tf22");
  visual_tools.trigger(); // to apply changes


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");




  geometry_msgs::TransformStamped base_link_to_leap_motion3; // My frames are named "base_link" and "leap_motion"

  base_link_to_leap_motion3 = tf_buffer.lookupTransform( "link_6", collision_object_baris[0].id, ros::Time(0), ros::Duration(1.0) );


  tf2::doTransform(map_pose[collision_object_baris[0].id], grasps[0].grasp_pose.pose, base_link_to_leap_motion3); // robot_pose is the PoseStamped I want to transform


  std::cout<< grasps[0].grasp_pose.pose <<std::endl;

  visual_tools.publishAxisLabeled(grasps[0].grasp_pose.pose, "bari_pose_tf3");
  visual_tools.trigger(); // to apply changes


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");




  // Show text in RViz of status


  // grasps[0].grasp_pose.pose.position.x = 0.415;
  // grasps[0].grasp_pose.pose.position.y = 0;
  // grasps[0].grasp_pose.pose.position.z = 0.5;


  // grasps[0].grasp_pose.pose.orientation.x = 0.93373;
  // grasps[0].grasp_pose.pose.orientation.y = -0.35765;
  // grasps[0].grasp_pose.pose.orientation.z = 0.0057657;
  // grasps[0].grasp_pose.pose.orientation.w = 0.014457;
  // grasps[0].grasp_pose.pose.position.x = 0.5+ 0.04;
  // grasps[0].grasp_pose.pose.position.y = -0.05 -0.04+ 0.05;
  // grasps[0].grasp_pose.pose.position.z = 0.53731+0.02;


  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "link_tool";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  // grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  // /* Direction is set as positive z axis */
  // grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  // grasps[0].post_grasp_retreat.min_distance = 0.1;
  // grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  //openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  //closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  //move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  move_group.pick("bari0", grasps);
  // END_SUB_TUTORIAL
}





void full_scenario_pick_place(moveit::planning_interface::MoveGroupInterface &move_group_interface, std::vector<moveit_msgs::CollisionObject> &collision_object_baris, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface ,geometry_msgs::PoseStamped scan_pose, moveit_visual_tools::MoveItVisualTools &visual_tools, const moveit::core::JointModelGroup* joint_model_group, std::vector<geometry_msgs::PoseStamped> &final_poses, std::vector<geometry_msgs::PoseStamped> &bari_poses, int M, int N){

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.3;

  std::chrono::seconds dura70(70);
  
  //trajecto_initial_to_scan_and_bari( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses);
  trajecto_initial_to_scan_and_bari2( move_group_interface, collision_object_baris, planning_scene_interface , scan_pose, visual_tools, joint_model_group, bari_poses);

  setup_planner(move_group_interface);


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  pick(move_group_interface, collision_object_baris, planning_scene_interface, visual_tools);

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


  // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
  // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
  //ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", 0);
  //move_group_interface.attachObject(collision_object_baris[0].id, "link_tool");

  visual_tools.publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools.trigger();


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
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot : %d", 0);
  move_group_interface.detachObject(collision_object_baris[0].id);

  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

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

  geometry_msgs::PoseStamped scan_pose;
  scan_pose.header.frame_id = "base_link";

  // scan_pose.orientation.x = 0.93373;
  // scan_pose.orientation.y = -0.35765;
  // scan_pose.orientation.z = 0.0057657;
  // scan_pose.orientation.w = 0.014457;
  // scan_pose.position.x = 0.75;
  // scan_pose.position.y = 0.0;
  // scan_pose.position.z = 0.5;
  scan_pose.pose.orientation.x = -0.29956;
  scan_pose.pose.orientation.y = 0.65046;
  scan_pose.pose.orientation.z = -0.25386;
  scan_pose.pose.orientation.w = 0.65018;
  scan_pose.pose.position.x = -0.089795;
  scan_pose.pose.position.y = -0.71612;
  scan_pose.pose.position.z = 0.3414;


  std::vector<geometry_msgs::PoseStamped> bari_poses;
  std::vector<geometry_msgs::PoseStamped> final_poses;


  int N_bari_x = 2;
  int N_bari_y = 4;
  int N_bari_z = 1;
  for (int xi = 0; xi< N_bari_x;xi++){
    for (int yi = 0; yi< N_bari_y;yi++){
      for (int zi = 0; zi< N_bari_z;zi++){


        geometry_msgs::PoseStamped target_pose_final;
        target_pose_final.header.frame_id = "base_link";
        target_pose_final.pose.orientation.x = 0.93373;
        target_pose_final.pose.orientation.y = -0.35765;
        target_pose_final.pose.orientation.z = 0.0057657;
        target_pose_final.pose.orientation.w = 0.014457;
        target_pose_final.pose.position.x = 0.22 + 0.1*yi;
        target_pose_final.pose.position.y = -0.80 + 0.05*xi;
        target_pose_final.pose.position.z = 0.4;
        // target_pose_final.orientation.x = -0.65663;
        // target_pose_final.orientation.y = 0.25469;
        // target_pose_final.orientation.z = 0.25726;
        // target_pose_final.orientation.w = 0.66166;
        // target_pose_final.position.x = 0.48;
        // target_pose_final.position.y = -0.05;
        // target_pose_final.position.z = 0.51;

        final_poses.push_back(target_pose_final);


        geometry_msgs::PoseStamped bari_pose;
        bari_pose.header.frame_id = "base_link";

        bari_pose.pose.orientation.x = 0.93373;
        bari_pose.pose.orientation.y = -0.35765;
        bari_pose.pose.orientation.z = 0.0057657;
        bari_pose.pose.orientation.w = 0.014457;
        bari_pose.pose.position.x = 0.5+ 0.04*xi;
        bari_pose.pose.position.y = -0.05 -0.04+ 0.05*yi;
        bari_pose.pose.position.z = 0.53731+0.02;
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

  std::vector<std::vector<geometry_msgs::PoseStamped>> vec_grasping_poses;
  std::vector<geometry_msgs::PoseStamped> grasping_poses;
  std::vector<geometry_msgs::PoseStamped> grasping_poses_true;

  //TODO CHANGE
  std::ifstream fichier("/home/guillaume/Téléchargements/somfyBarrel.txt");
  //std::ifstream fichier("/home/guillaume/Téléchargements/rolling.txt");

  if(fichier)
  {
    //L'ouverture s'est bien passée, on peut donc lire

    std::string ligne; //Une variable pour stocker les lignes lues
    getline(fichier, ligne);
    getline(fichier, ligne);
    getline(fichier, ligne);
    getline(fichier, ligne);
    getline(fichier, ligne);
    getline(fichier, ligne);
    getline(fichier, ligne);
    getline(fichier, ligne);


    while(getline(fichier, ligne)) //Tant qu'on n'est pas à la fin, on lit
    {
        std::stringstream ss(ligne);

        std::cout << ligne << std::endl;
        std::string tx, ty, tz, rx, ry, rz;
        std::getline(ss,tx,',');    
        std::cout<<"\""<<tx<<"\""<<std::endl;
        std::getline(ss,ty,','); 
        std::cout<<", \""<<ty<<"\""<<std::endl;
        std::getline(ss,tz,','); 
        std::cout<<", \""<<tz<<"\""<<std::endl;
        std::getline(ss,rx,',');    
        std::cout<<"\""<<rx<<"\""<<std::endl;
        std::getline(ss,ry,','); 
        std::cout<<", \""<<ry<<"\""<<std::endl;
        std::getline(ss,rz,','); 
        std::cout<<", \""<<rz<<"\""<<std::endl;
        Eigen::Isometry3d tf_tcp_in_bari;
        tf_tcp_in_bari = create_iso_tcp_in_bari(std::stof(tx), std::stof(ty), std::stof(tz), std::stof(rx), std::stof(ry), std::stof(rz));
        geometry_msgs::PoseStamped tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, 0.1);
        grasping_poses.push_back(tf_transformed);
        tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, 0.00);
        grasping_poses_true.push_back(tf_transformed);
        //Et on l'affiche dans la console 
        //Ou alors on fait quelque chose avec cette ligne
        //À vous de voir
    }
  }
  else
  {
    std::cout << "ERREUR: Impossible d'ouvrir le fichier en lecture." << std::endl;
  }





  // Eigen::Isometry3d tf_tcp_in_bari;
  // tf_tcp_in_bari = create_iso_tcp_in_bari(0.0, -0.011, -0.001, -90, 0, 90);
  // geometry_msgs::PoseStamped tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, 0.05);
  // grasping_poses.push_back(tf_transformed);
  // tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, 0.00);
  // grasping_poses_true.push_back(tf_transformed);
  // tf_tcp_in_bari = create_iso_tcp_in_bari(0.0, -0.011, -0.001, -90, 0, -90);
  // tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, 0.05);
  // grasping_poses.push_back(tf_transformed);
  // tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, 0.00);
  // grasping_poses_true.push_back(tf_transformed);
  std::cout<<grasping_poses.size()<<std::endl;
  vec_grasping_poses.push_back(grasping_poses);
  vec_grasping_poses.push_back(grasping_poses_true);


  //tf_transformed.header.frame_id = "bari0";



  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  std::vector<moveit_msgs::CollisionObject> collision_object_baris;

  int M = 2;
  int N = 2; //(+1 bari detecté par scan)



  //full_scenario_without_attach( move_group_interface, collision_object_baris, planning_scene_interface, scan_pose, visual_tools, joint_model_group, final_poses, bari_poses, M, N);

  //full_scenario( move_group_interface, collision_object_baris, planning_scene_interface, scan_pose, visual_tools, joint_model_group, final_poses, bari_poses, grasping_poses, M, N);

  full_scenario_grasp( move_group_interface, collision_object_baris, planning_scene_interface, scan_pose, visual_tools, joint_model_group, final_poses, bari_poses, vec_grasping_poses, M, N);


  //full_scenario_pick_place( move_group_interface, collision_object_baris, planning_scene_interface, scan_pose, visual_tools, joint_model_group, final_poses, bari_poses, M, N);


  //scenario_test_scan_attach( move_group_interface, collision_object_baris, planning_scene_interface, scan_pose, visual_tools, joint_model_group, final_poses, bari_poses, M, N);


  //ok scenario_test_out_attach( move_group_interface, collision_object_baris, planning_scene_interface, scan_pose, visual_tools, joint_model_group, final_poses, bari_poses, M, N);




  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<<"Global time to find plan : "<<Global_time_find_planning<<" s "<<std::endl;
  std::cout<<"Global time to execute traj : "<<Global_time_traj<<" s "<<std::endl;
  std::cout<<"Global nb of step in the traj : "<<Global_nb_step_traj<<std::endl;
  std::cout<<"Global move of the tool : "<<Global_move_tool<< " m"<<std::endl;
  std::cout<<"Nombre de fail : "<< NB_fail<< std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;



  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
