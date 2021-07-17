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



class Moveit_Engine

{

public:

  Moveit_Engine();
  Moveit_Engine(int argc, char** argv);
  Moveit_Engine(int argc, char** argv, moveit::planning_interface::MoveGroupInterface move_group_interface, moveit::planning_interface::PlanningSceneInterface planning_scene_interface,const moveit::core::JointModelGroup* joint_model_group);
  ~Moveit_Engine();


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

  tf2::Quaternion set_quentin(float rx, float ry, float rz){
    tf2::Quaternion q_rotx;
    q_rotx.setRotation(tf2::Vector3(1,0,0),rx*M_PI/180.0);

    tf2::Quaternion q_roty;
    q_roty.setRotation(tf2::Vector3(0,1,0),ry*M_PI/180.0);

    tf2::Quaternion q_rotz;
    q_rotz.setRotation(tf2::Vector3(0,0,1),rz*M_PI/180.0);

    return q_rotx*q_roty*q_rotz;
  }

  void fill_vector_cin(std::vector<double>& boxInsideSizeM, std::string info){

    std::string line = "";
    double number;
    while (line == ""){
      std::cout << info << std::endl;
      std::cout << "Enter numbers separated by spaces: ";
      std::getline(std::cin, line);
      std::cout << line << std::endl;
      std::istringstream stream(line);
      while (stream >> number){
        std::cout<<"nombre"<<number<<std::endl;
        boxInsideSizeM.push_back(number);
      }
    }
  }

  void fill_vector_cin_angle(std::vector<double>& boxInsideSizeM, std::string info){

    std::string line = "";
    double number;
    while (line == ""){
      std::cout << info << std::endl;
      std::cout << "Enter numbers separated by spaces: ";
      std::getline(std::cin, line);
      std::cout << line << std::endl;
      std::istringstream stream(line);
      while (stream >> number){
        std::cout<<"nombre"<<number<<std::endl;
        boxInsideSizeM.push_back(number*M_PI/180.0);
      }
    }
  }


  geometry_msgs::PoseStamped get_pose(std::string info){
    std::vector<double> vector_quentin;// = {0.269572 0.184798 0.177178};

    fill_vector_cin(vector_quentin, info);

    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = "base";
    tf2::Quaternion quat;
    std::cout << "vector_quentin" << vector_quentin[3] << " " << vector_quentin[4] << " " << vector_quentin[5] << std::endl;
    quat = set_quentin(vector_quentin[3],vector_quentin[4],vector_quentin[5]);
    tf2::convert(quat, pose.pose.orientation); 


    std::cout << "pose.pose.orientation" << pose.pose.orientation.x << " " << pose.pose.orientation.y << " " << pose.pose.orientation.z << " " << pose.pose.orientation.w << std::endl;

    pose.pose.position.x = vector_quentin[0];
    pose.pose.position.y = vector_quentin[1];
    pose.pose.position.z = vector_quentin[2];

    geometry_msgs::PoseStamped base_link_moveit_to_halcon;
    base_link_moveit_to_halcon.pose.position.x = 0.0;
    base_link_moveit_to_halcon.pose.position.y = 0.0;
    base_link_moveit_to_halcon.pose.position.z = 0.478;

    Eigen::Isometry3d tf_base_link_moveit_to_halcon;
    tf2::fromMsg(base_link_moveit_to_halcon.pose, tf_base_link_moveit_to_halcon); //pose in bary frame

    Eigen::Isometry3d tf_pose;
    tf2::fromMsg(pose.pose, tf_pose);

    //pose.pose = tf2::toMsg(tf_base_link_moveit_to_halcon * tf_pose); //world to bary * pose in bary = pose in world
    pose.pose = tf2::toMsg( tf_pose);

    return pose;

  }

  void load_bari_in_scene_simulation(std::vector<moveit_msgs::CollisionObject> &collision_object_baris){


    // TODO CHANGE
    //std::string modelpath = "/home/pc-m/Documents/My-cao/barillet.obj";
    //std::string modelpath = "package://geometric_shapes/test/resources/5067976_barillet_005.obj";
    std::string modelpath = "package://geometric_shapes/test/resources/barillet_convex.obj";
    shape_msgs::Mesh mesh_bary = load_mesh(modelpath);


    int N_bari_x = 2;
    int N_bari_y = 4;
    int N_bari_z = 1;
    for (int xi = 0; xi< N_bari_x;xi++){
      for (int yi = 0; yi< N_bari_y;yi++){
        for (int zi = 0; zi< N_bari_z;zi++){

          char id_bari[20];
          sprintf(id_bari,"bari%d",xi+N_bari_x*yi);
          std::string str_id_obj=id_bari;

          // Define a pose for the box (specified relative to frame_id)
          geometry_msgs::PoseStamped pose;

          tf2::Quaternion q_rot;

          // TODO CHANGE
          //q_rot.setRPY(3.14,0.00,0.00); //rk
          //q_rot.setRPY(1.57,3.14,-3.14); //bari
          //q_rot.setRPY(1.60,1.58,-3.14); //bari
          //q_rot.setRPY(1.57,2.20,2.16);  //bari_rot
          //q_rot.setRPY(-1.15,1.57,0.0);  //bari_rot
          //q_rot = random_quaternion();

          // Stuff the new rotation back into the pose. This requires conversion into a msg type
          //tf2::convert(q_rot, pose.orientation);
          pose.pose.orientation.x = -0.5;
          pose.pose.orientation.y = 0.5;
          pose.pose.orientation.z = -0.5;
          pose.pose.orientation.w = 0.5;

          // pose.orientation.x = -0.3842906;
          // pose.orientation.y = 0.5932874;
          // pose.orientation.z = -0.3848079;
          // pose.orientation.w = 0.5935097;

          // pose.pose.orientation.x = 0; //bari prise 16
          // pose.pose.orientation.y = 0.7071068;
          // pose.pose.orientation.z = 0;
          // pose.pose.orientation.w = 0.7071068;
          

          //double res_x = center.x -0.5 + 0.1*xi;
          pose.pose.position.x = -0.08+ 0.05*yi;
          pose.pose.position.y = -0.74+ 0.04*xi;
          pose.pose.position.z = 0.06;

          pose.header.frame_id = "base_link";

          load_obj_in_scene( collision_object_baris, mesh_bary, pose, str_id_obj);

        }
      }
    }
    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
    //ROS_INFO_NAMED("tutorial", "Add an object into the world");
    //planning_scene_interface_ptr->addCollisionObjects(collision_object_baris);


  }


  shape_msgs::Mesh load_mesh(std::string modelpath){

    ROS_INFO("mesh loades : %s ",modelpath.c_str());

    static const Eigen::Vector3d scale(0.001, 0.001, 0.001);
    shapes::Mesh* cao_bary = shapes::createMeshFromResource(modelpath,scale);

    ROS_INFO("mesh loades : %i triangles : %s ",cao_bary->triangle_count,modelpath.c_str());


    shape_msgs::Mesh mesh_bary;
    shapes::ShapeMsg mesh_bary_msg;

    shapes::constructMsgFromShape(cao_bary,mesh_bary_msg);
    mesh_bary = boost::get<shape_msgs::Mesh>(mesh_bary_msg);

    return (mesh_bary);
    
  }

  void load_obj_in_scene( std::vector<moveit_msgs::CollisionObject> &collision_object_baris, shape_msgs::Mesh mesh_bary, geometry_msgs::PoseStamped pose, std::string str_id_obj){


    // Now let's define a collision object ROS message for the robot to avoid.
    moveit_msgs::CollisionObject collision_object_bari;

    //char id_bari[20];
    //sprintf(id_bari,"bari%d",xi+N_bari_x*yi);

    // The id of the object is used to identify it.
    collision_object_bari.id = str_id_obj;


    //tf2::Quaternion q_rot;

    // TODO CHANGE
    //q_rot.setRPY(3.14,0.00,0.00); //rk
    //q_rot.setRPY(1.57,3.14,-3.14); //bari
    //q_rot.setRPY(1.60,1.58,-3.14); //bari
    //q_rot.setRPY(1.57,2.20,2.16);  //bari_rot
    //q_rot = random_quaternion();

    // Stuff the new rotation back into the pose. This requires conversion into a msg type
    //tf2::convert(q_rot, pose.orientation);
    // pose.orientation.x = -0.5;
    // pose.orientation.y = 0.5;
    // pose.orientation.z = -0.5;
    // pose.orientation.w = 0.5;

    // //double res_x = center.x -0.5 + 0.1*xi;
    // pose.position.x = -0.08+ 0.05*yi;
    // pose.position.y = -0.74+ 0.04*xi;
    // pose.position.z = 0.06;
    collision_object_bari.header.frame_id = pose.header.frame_id;

    collision_object_bari.meshes.push_back(mesh_bary);
    collision_object_bari.mesh_poses.push_back(pose.pose);
    collision_object_bari.operation = collision_object_bari.ADD;

    collision_object_baris.push_back(collision_object_bari);

    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface_ptr->addCollisionObjects(collision_object_baris);


  }


  void load_carton_in_scene(std::vector<moveit_msgs::CollisionObject> &collision_object_baris){

    //std::string modelpath = "/home/pc-m/Documents/My-cao/barillet.obj";
    std::string modelpath = "package://geometric_shapes/test/resources/Contenant_barillet.stl";
    shape_msgs::Mesh mesh_bary = load_mesh(modelpath);

    // Now let's define a collision object ROS message for the robot to avoid.
    moveit_msgs::CollisionObject collision_object_bari;

    // The id of the object is used to identify it.
    std::string str_id_obj = "carton";

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::PoseStamped pose;

    tf2::Quaternion q_rot;

    // TODO CHANGE
    //q_rot.setRPY(3.14,0.00,0.00); //rk
    //q_rot.setRPY(1.57,3.14,-3.14); //bari
    q_rot.setRPY(1.57,0.0,0.0); //bari
    //q_rot.setRPY(0.0,0.0,0.0); //bari prise 13
    //q_rot.setRPY(1.57,2.20,2.16);  //bari_rot
    //q_rot = random_quaternion();

    // Stuff the new rotation back into the pose. This requires conversion into a msg type
    tf2::convert(q_rot, pose.pose.orientation);


    pose.pose.position.x = 0.0;
    pose.pose.position.y = -0.7;
    pose.pose.position.z = 0.2;

    pose.header.frame_id = "base_link";

    load_obj_in_scene(collision_object_baris, mesh_bary, pose, str_id_obj);





  }


  void create_carton_in_scene( std::vector<moveit_msgs::CollisionObject> &collision_object_baris, std::vector<double> boxInsideSizeM ,std::vector<double> boxInsidePoseM , std::vector<double> boxThickSideBottomM){

    geometry_msgs::PoseStamped base_link_moveit_to_halcon;
    base_link_moveit_to_halcon.pose.position.x = 0.0;
    base_link_moveit_to_halcon.pose.position.y = 0.0;
    base_link_moveit_to_halcon.pose.position.z = 0.478;

    Eigen::Isometry3d tf_base_link_moveit_to_halcon;
    tf2::fromMsg(base_link_moveit_to_halcon.pose, tf_base_link_moveit_to_halcon); //pose in bary frame


    geometry_msgs::PoseStamped tf_to_scene;

    tf2::Quaternion q_rot_box;
    q_rot_box = set_quentin(boxInsidePoseM[3], boxInsidePoseM[4], boxInsidePoseM[5]);
    tf2::convert(q_rot_box, tf_to_scene.pose.orientation);

    tf_to_scene.pose.position.x = boxInsidePoseM[0];
    tf_to_scene.pose.position.y = boxInsidePoseM[1];
    tf_to_scene.pose.position.z = boxInsidePoseM[2];

    Eigen::Isometry3d tf_to_scene_tot;
    tf2::fromMsg(tf_to_scene.pose, tf_to_scene_tot); //pose in bary frame





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
    q_rot = set_quentin(-90,0.0,0.0); //bari

    // Stuff the new rotation back into the pose. This requires conversion into a msg type
    tf2::convert(q_rot, pose.orientation);

    pose.position.x = 0.0;
    pose.position.y = -(boxInsideSizeM[1] / 2.0 + boxThickSideBottomM[0] / 2.0);
    pose.position.z = 0.0;

    Eigen::Isometry3d tf_carton;
    tf2::fromMsg(pose, tf_carton); //pose in bary frame

    pose = tf2::toMsg(tf_base_link_moveit_to_halcon * tf_to_scene_tot * tf_carton); //world to bary * pose in bary = pose in world



    collision_object_box.primitives.push_back(primitive);
    collision_object_box.primitive_poses.push_back(pose);
    collision_object_box.operation = collision_object_box.ADD;

    collision_object_baris.push_back(collision_object_box);
  //////

    collision_object_box.header.frame_id = "base_link";
    // The id of the object is used to identify it.
    collision_object_box.id = "carton2";
    q_rot = set_quentin(-90,0.0,0.0); //bari

    // Stuff the new rotation back into the pose. This requires conversion into a msg type
    tf2::convert(q_rot, pose.orientation);

    pose.position.x = 0.0;
    pose.position.y = (boxInsideSizeM[1] / 2.0 + boxThickSideBottomM[0] / 2.0);
    pose.position.z = 0.0;

    tf2::fromMsg(pose, tf_carton); //pose in bary frame
    pose = tf2::toMsg(tf_base_link_moveit_to_halcon * tf_to_scene_tot * tf_carton); //world to bary * pose in bary = pose in world


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

    q_rot = set_quentin(0.0,-90,0.0); //bari

    // Stuff the new rotation back into the pose. This requires conversion into a msg type
    tf2::convert(q_rot, pose.orientation);

    pose.position.x = (boxInsideSizeM[0] / 2.0 + boxThickSideBottomM[0] / 2.0);
    pose.position.y = 0.0;
    pose.position.z = 0.0;

    tf2::fromMsg(pose, tf_carton); //pose in bary frame
    pose = tf2::toMsg(tf_base_link_moveit_to_halcon * tf_to_scene_tot * tf_carton); //world to bary * pose in bary = pose in world

    collision_object_box.primitives.push_back(primitive);
    collision_object_box.primitive_poses.push_back(pose);
    collision_object_box.operation = collision_object_box.ADD;

    collision_object_baris.push_back(collision_object_box);
  //////

    collision_object_box.header.frame_id = "base_link";
    // The id of the object is used to identify it.
    collision_object_box.id = "carton4";
    q_rot = set_quentin(0.0,-90,0.0); //bari

    // Stuff the new rotation back into the pose. This requires conversion into a msg type
    tf2::convert(q_rot, pose.orientation);

    pose.position.x = -(boxInsideSizeM[0] / 2.0 + boxThickSideBottomM[0] / 2.0);
    pose.position.y = 0.0;
    pose.position.z = 0.0;

    tf2::fromMsg(pose, tf_carton); //pose in bary frame
    pose = tf2::toMsg(tf_base_link_moveit_to_halcon * tf_to_scene_tot * tf_carton); //world to bary * pose in bary = pose in world

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
    pose = tf2::toMsg(tf_base_link_moveit_to_halcon * tf_to_scene_tot * tf_carton); //world to bary * pose in bary = pose in world


    collision_object_box.primitives.push_back(primitive);
    collision_object_box.primitive_poses.push_back(pose);
    collision_object_box.operation = collision_object_box.ADD;

    collision_object_baris.push_back(collision_object_box);


    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface_ptr->addCollisionObjects(collision_object_baris);


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


 void set_joint_constraint(std::vector<float> above_zero, std::vector<float> below_zero){
    moveit_msgs::Constraints test_constraints;

    for (int i = 0; i < above_zero.size()-1; i++){

      moveit_msgs::JointConstraint jc1;
      jc1.joint_name = "joint_" + std::to_string(i+1);  
      std::cout << jc1.joint_name << std::endl;
      jc1.position = 0.0;
      jc1.tolerance_above = above_zero[i];//0.0; //80 degrés
      jc1.tolerance_below = -below_zero[i];//3.0;
      jc1.weight = 1.0; 

      std::cout << "joint_name_constrain : " << "joint_" + std::to_string(i+1) << "[" << -below_zero[i] << ", " << above_zero[i] << "]" << std::endl;

      test_constraints.joint_constraints.push_back(jc1);

    }

    move_group_interface_ptr->setPathConstraints(test_constraints);

  }


  // moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(&isIKStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
    //                           collision_checking_verbose_, only_check_self_collision_, visual_tools_, _1, _2, _3);

  void setup_planner(int Nb_attempt, double time_limit, std::string planner_id){

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

    std::cout << "Nb_attempt : " << Nb_attempt << std::endl;
    std::cout << "time_limit : " << time_limit << std::endl;
    std::cout << "planner_id : " << planner_id << std::endl;


    move_group_interface_ptr->setPlannerId(planner_id);
    std::map<std::string, std::string> parametre = move_group_interface_ptr->getPlannerParams(planner_id,"arm_group");

    std::map<std::string, std::string>::iterator it;
    for (it = parametre.begin(); it != parametre.end(); it++)
    {
      std::cout << it->first    // string (key)
      << " : "
      << it->second   // string's value 
      << std::endl;
    }
    //std::cout << "parametre[type] : "<< parametre["type"] << std::endl;
    //parametre["type"] = "geometric::LazyPRM";
    //std::cout << "parametre[type] : "<< parametre["type"] << std::endl;

    //move_group_interface.setPlannerParams(planner_id, "arm_group", parametre);

    move_group_interface_ptr->setNumPlanningAttempts(Nb_attempt);

    move_group_interface_ptr->setPlanningTime(time_limit);
    /** \brief Set a scaling factor for optionally reducing the maximum joint velocity.
        Allowed values are in (0,1]. The maximum joint velocity specified
        in the robot model is multiplied by the factor. If the value is 0, it is set to
        the default value, which is defined in joint_limits.yaml of the moveit_config.
        If the value is greater than 1, it is set to 1.0. */
    move_group_interface_ptr->setMaxVelocityScalingFactor(1.0);

    /** \brief Set a scaling factor for optionally reducing the maximum joint acceleration.
        Allowed values are in (0,1]. The maximum joint acceleration specified
        in the robot model is multiplied by the factor. If the value is 0, it is set to
        the default value, which is defined in joint_limits.yaml of the moveit_config.
        If the value is greater than 1, it is set to 1.0. */
    move_group_interface_ptr->setMaxAccelerationScalingFactor(1.0);






    moveit_msgs::PositionConstraint pcm;
    pcm.link_name = "link_tool";
    pcm.header.frame_id = "base_link";

    shape_msgs::SolidPrimitive work_box;
    work_box.type = work_box.BOX;
    work_box.dimensions.resize(3);
    work_box.dimensions[work_box.BOX_X] = 0.9; 
    work_box.dimensions[work_box.BOX_Y] = 0.6; 
    work_box.dimensions[work_box.BOX_Z] = 0.8; 

    geometry_msgs::Pose work_box_pose;
    work_box_pose.orientation.w = 1.0;
    work_box_pose.position.y = -0.7;
    work_box_pose.position.x = 0.25;
    work_box_pose.position.z = 0.3;

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


    std::vector<float> above = {0.0, 1.57, 2.53, 2.27, 2.39, 3.14};
    std::vector<float> below = {-3.0, -0.7, -0.7, -2.27, -2, -3.15};

    set_joint_constraint(above, below);

  }

 

  EigenSTL::vector_Vector3d evaluate_plan(moveit::planning_interface::MoveGroupInterface::Plan &plan,double &local_time_find_plan, double &time_traj, robot_trajectory::RobotTrajectory &trajecto_state){



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
    //std::cout << "test std::cout" << trajecto_state.getWayPoint(1) << std::endl;  //trajecto_state.getWayPointCount()

    //std::cout << "Link poses:" << std::endl;



    EigenSTL::vector_Vector3d path;

    double local_move_tool = 0;
    double local_angle_move_tool = 0;
  
    for (std::size_t i = 1; i < trajecto_state.getWayPointCount(); ++i)
    {
      const Eigen::Isometry3d transform0 =trajecto_state.getWayPoint(i-1).getGlobalLinkTransform("link_finger1");
      const Eigen::Isometry3d transform1 =trajecto_state.getWayPoint(i).getGlobalLinkTransform("link_finger1");

      ASSERT_ISOMETRY(transform0)  // unsanitized input, could contain a non-isometry
      ASSERT_ISOMETRY(transform1)

      path.push_back(transform0.translation());
      visual_tools_ptr->publishSphere(transform0, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
      //Eigen::Quaterniond q(transform.linear());
      //std::cout << "T.xyz = [" << transform.translation().x() << ", " << transform.translation().y() << ", "
      //<< transform.translation().z() << "], Q.xyzw = [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w()
      //<< "]" << std::endl;

      //std::cout << "TYPE!! : "<< typeid(transform.translation()).name() << std::endl;

      //double dist = distance((double) transform.translation().x(),(double) transform.translation().y(),(double) transform.translation().z(),(double) transform.translation().x(),(double) transform.translation().y(),(double) transform.translation().z()); 
      double dist = distance((double) transform0.translation().x(),(double) transform0.translation().y(),(double) transform0.translation().z(),(double) transform1.translation().x(),(double) transform1.translation().y(),(double) transform1.translation().z()); 
      
      std::vector<double> Joint_trajecto;
      trajecto_state.getWayPoint(i-1).copyJointGroupPositions(joint_model_group_ptr, Joint_trajecto);
      std::vector<double> Joint_trajecto2;
      trajecto_state.getWayPoint(i).copyJointGroupPositions(joint_model_group_ptr, Joint_trajecto2);

      double dist6D = distance6D((double) Joint_trajecto[0]*180/M_PI,(double) Joint_trajecto[1]*180/M_PI,(double) Joint_trajecto[2]*180/M_PI,
                                 (double) Joint_trajecto[3]*180/M_PI,(double) Joint_trajecto[4]*180/M_PI,(double) Joint_trajecto[5]*180/M_PI,
                                 (double) Joint_trajecto2[0]*180/M_PI,(double) Joint_trajecto2[1]*180/M_PI,(double) Joint_trajecto2[2]*180/M_PI,
                                 (double) Joint_trajecto2[3]*180/M_PI, (double) Joint_trajecto2[4]*180/M_PI, (double) Joint_trajecto2[5]*180/M_PI);

      Global_move_tool += dist;
      Global_angle_move_tool += dist6D;
      local_move_tool += dist;
      local_angle_move_tool += dist6D;



    }


    std::cout<< "###########" << std::endl;
    std::cout<< "###########" << std::endl;
    std::cout<< "DIST TOOL FOR TRAJ " << NUM_TRAJ << " : " << local_move_tool << "m" <<std::endl;
    std::cout<< "###########" << std::endl;
    std::cout<< "###########" << std::endl;


    std::cout<< "###########" << std::endl;
    std::cout<< "###########" << std::endl;
    std::cout<< "DIST ANGLE FOR TRAJ " << NUM_TRAJ << " : " << local_angle_move_tool << "m" <<std::endl;
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
    q_test333 = set_quentin( 20, 0.0, 0.0); //20°

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
    q_test444 = set_quentin( 0.0, 0.0, -45); //45°

    tf2::convert(q_test444, tf_link6_to_tool.pose.orientation);

    //double res_x = center.x -0.5 + 0.1*xi;
    tf_link6_to_tool.pose.position.x = 0.0;
    tf_link6_to_tool.pose.position.y = 0.0;
    tf_link6_to_tool.pose.position.z = 0.0;
    Eigen::Isometry3d tf_iso_link6_to_tool;
    tf2::fromMsg(tf_link6_to_tool.pose, tf_iso_link6_to_tool); //pose in bary frame



    pose_transformed.pose = tf2::toMsg( tf_tcp_in_bari * tf_iso_translation * tf_iso_tool_to_tcp.inverse() * tf_iso_link6_to_tool.inverse()); //world to bary * pose in bary = pose in world


    return pose_transformed;
  }

  // translation(m) / Rotation (deg)
  // Poses defined in object's frame (x,y,z,rx,ry,rz)
  Eigen::Isometry3d create_iso_tcp_in_bari(float tx, float ty, float tz, float rx, float ry, float rz){

    geometry_msgs::PoseStamped tcp_in_bari;
    tf2::Quaternion q_test;
    q_test = set_quentin(rx, ry, rz);
    tf2::convert(q_test, tcp_in_bari.pose.orientation);

    //double res_x = center.x -0.5 + 0.1*xi;
    tcp_in_bari.pose.position.x = tx;//0.0;
    tcp_in_bari.pose.position.y = ty;//-0.011;
    tcp_in_bari.pose.position.z = tz;//-0.001;
    Eigen::Isometry3d tf_tcp_in_bari;
    tf2::fromMsg(tcp_in_bari.pose, tf_tcp_in_bari); //pose in bary frame

    return tf_tcp_in_bari;

  }

  std::vector<std::vector<double>> go_to_position(std::vector<double> target_pose){


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


    //showFrames(target_pose, target_pose.header.frame_id);
    visual_tools_ptr->publishText(text_pose, "Show target frame", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger(); // to apply changes
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

    std::cout << target_pose[0] <<std::endl;
    std::cout << target_pose[1] <<std::endl;
    std::cout << target_pose[2] <<std::endl;
    std::cout << target_pose[3] <<std::endl;
    std::cout << target_pose[4] <<std::endl;
    std::cout << target_pose[5] <<std::endl;

    //visual_tools_ptr->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



    move_group_interface_ptr->setJointValueTarget(target_pose);



    // /** \brief Get the current joint state goal in a form compatible to setJointValueTarget() */
    // void getJointValueTarget(std::vector<double>& group_variable_values) const;


    std::string planner_test = move_group_interface_ptr->getPlannerId();
    std::cout<<"getPlannerId : "<<planner_test<<std::endl;


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    double local_time_find_plan = 0;
    bool success = false;
    for (int i=0; i < 10 ;i++){
      success = (move_group_interface_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success) {
        local_time_find_plan += my_plan.planning_time_;
        break;
      }
      else {
        local_time_find_plan += move_group_interface_ptr->getPlanningTime();
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
    move_group_interface_ptr->execute(my_plan);
    std::chrono::high_resolution_clock::time_point end_traj = std::chrono::high_resolution_clock::now();

    double time_traj = std::chrono::duration_cast<std::chrono::microseconds>(end_traj-begin_traj).count()/1000000.0;

    robot_trajectory::RobotTrajectory trajecto_state(move_group_interface_ptr->getCurrentState()->getRobotModel(), move_group_interface_ptr->getName());

    trajecto_state.setRobotTrajectoryMsg(*move_group_interface_ptr->getCurrentState(), my_plan.trajectory_);

    std::vector<std::vector<double>> trajecto_waypoint_joint;
    for (std::size_t i = 1; i < trajecto_state.getWayPointCount(); ++i)
    {

      std::vector<double> waypoint_joint;
      trajecto_state.getWayPointPtr(i)->copyJointGroupPositions(joint_model_group_ptr, waypoint_joint);
      trajecto_waypoint_joint.push_back(waypoint_joint);
    }





    EigenSTL::vector_Vector3d path = evaluate_plan(my_plan, local_time_find_plan, time_traj, trajecto_state);

    const double radius = 0.005;
    visual_tools_ptr->publishPath(path, rviz_visual_tools::GREEN, radius);
    visual_tools_ptr->trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



    return trajecto_waypoint_joint;
  }

  std::vector<std::vector<double>> go_to_position(geometry_msgs::PoseStamped target_pose ){


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
    visual_tools_ptr->publishText(text_pose, "Show target frame", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger(); // to apply changes
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

    move_group_interface_ptr->setPoseTarget(target_pose);



    // /** \brief Get the current joint state goal in a form compatible to setJointValueTarget() */
    // void getJointValueTarget(std::vector<double>& group_variable_values) const;


    std::string planner_test = move_group_interface_ptr->getPlannerId();
    std::cout<<"getPlannerId : "<<planner_test<<std::endl;


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    double local_time_find_plan = 0;
    bool success = false;
    for (int i=0; i < 10 ;i++){
      success = (move_group_interface_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success) {
        local_time_find_plan += my_plan.planning_time_;
        break;
      }
      else {
        local_time_find_plan += move_group_interface_ptr->getPlanningTime();
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
    move_group_interface_ptr->execute(my_plan);
    std::chrono::high_resolution_clock::time_point end_traj = std::chrono::high_resolution_clock::now();

    double time_traj = std::chrono::duration_cast<std::chrono::microseconds>(end_traj-begin_traj).count()/1000000.0;

    robot_trajectory::RobotTrajectory trajecto_state(move_group_interface_ptr->getCurrentState()->getRobotModel(), move_group_interface_ptr->getName());

    trajecto_state.setRobotTrajectoryMsg(*move_group_interface_ptr->getCurrentState(), my_plan.trajectory_);

    std::vector<std::vector<double>> trajecto_waypoint_joint;
    for (std::size_t i = 1; i < trajecto_state.getWayPointCount(); ++i)
    {

      std::vector<double> waypoint_joint;
      trajecto_state.getWayPointPtr(i)->copyJointGroupPositions(joint_model_group_ptr, waypoint_joint);
      trajecto_waypoint_joint.push_back(waypoint_joint);
    }





    EigenSTL::vector_Vector3d path = evaluate_plan(my_plan, local_time_find_plan, time_traj, trajecto_state);

    const double radius = 0.005;
    visual_tools_ptr->publishPath(path, rviz_visual_tools::GREEN, radius);
    visual_tools_ptr->trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



    return trajecto_waypoint_joint;
  }


  std::vector<std::vector<double>> go_to_position(std::vector<geometry_msgs::PoseStamped> grasp_poses, int ID_grasp){


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
    //   visual_tools_ptr->publishText(text_pose, "Show target frame", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    //   visual_tools_ptr->trigger(); // to apply changes
    //   visual_tools_ptr->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    // }
    if (ID_grasp != -1) {



      showFrames(grasp_poses[ID_grasp], grasp_poses[ID_grasp].header.frame_id);
      visual_tools_ptr->publishText(text_pose, "Show target frame", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      visual_tools_ptr->trigger(); // to apply changes
      //visual_tools_ptr->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


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

      move_group_interface_ptr->setPoseTarget(grasp_poses[ID_grasp]);


    }
    else {
      move_group_interface_ptr->setPoseTargets(grasp_poses);
    }
    // /** \brief Get the current joint state goal in a form compatible to setJointValueTarget() */
    // void getJointValueTarget(std::vector<double>& group_variable_values) const;


    std::string planner_test = move_group_interface_ptr->getPlannerId();
    std::cout<<"getPlannerId : "<<planner_test<<std::endl;


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    double local_time_find_plan = 0;
    bool success = false;
    for (int i=0; i < 10 ;i++){
      success = (move_group_interface_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success) {
        local_time_find_plan += my_plan.planning_time_;
        break;
      }
      else {
        local_time_find_plan += move_group_interface_ptr->getPlanningTime();
        std::cout << "TRY AGAIN : " << i << std::endl;
        NB_fail++;
      }

    }

    if (!success) {
      std::cerr << "FAIL TO FIND A PATH AFTER 10 TRY AGAIN" << std::endl;
      exit(0);
    }

    //my_plan.trajectory_.

    std::chrono::high_resolution_clock::time_point begin_traj = std::chrono::high_resolution_clock::now();
    move_group_interface_ptr->execute(my_plan);
    std::chrono::high_resolution_clock::time_point end_traj = std::chrono::high_resolution_clock::now();

    double time_traj = std::chrono::duration_cast<std::chrono::microseconds>(end_traj-begin_traj).count()/1000000.0;

    robot_trajectory::RobotTrajectory trajecto_state(move_group_interface_ptr->getCurrentState()->getRobotModel(), move_group_interface_ptr->getName());

    trajecto_state.setRobotTrajectoryMsg(*move_group_interface_ptr->getCurrentState(), my_plan.trajectory_);

    std::vector<std::vector<double>> trajecto_waypoint_joint;
    for (std::size_t i = 1; i < trajecto_state.getWayPointCount(); ++i)
    {

      std::vector<double> waypoint_joint;
      trajecto_state.getWayPointPtr(i)->copyJointGroupPositions(joint_model_group_ptr, waypoint_joint);
      trajecto_waypoint_joint.push_back(waypoint_joint);
    }





    EigenSTL::vector_Vector3d path = evaluate_plan(my_plan, local_time_find_plan, time_traj, trajecto_state);

    const double radius = 0.005;
    visual_tools_ptr->publishPath(path, rviz_visual_tools::GREEN, radius);
    visual_tools_ptr->trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



    return trajecto_waypoint_joint;
  }

  void export_trajectory(std::vector<std::vector<double>> traj0){
    std::ofstream myfile ("/home/guillaume/ws_moveit/example.txt");
    if (myfile.is_open())
    {
      myfile << "end_header" << "\n" << "\n";
      for (int k=0; k<traj0.size(); k++){
        myfile << "f " << traj0[k][0]*180.0/M_PI << " " << traj0[k][1]*180.0/M_PI  << " " << traj0[k][2]*180.0/M_PI  << " " << traj0[k][3]*180.0/M_PI  << " " << traj0[k][4]*180.0/M_PI  << " " << traj0[k][5]*180.0/M_PI  << "\n";
      }
      myfile.close();
    }
    else std::cout << "Unable to open file";
  }

  std::vector<std::vector<double>> trajecto_approch( geometry_msgs::PoseStamped &pose_goal){

    

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

    move_group_interface_ptr->setStartStateToCurrentState();

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface_ptr->computeCartesianPath(vecULT, eef_step, jump_threshold, trajectory, false);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);



    visual_tools_ptr->trigger(); // to apply changes
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

    move_group_interface_ptr->execute(trajectory);

    visual_tools_ptr->trigger(); // to apply changes
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

    robot_trajectory::RobotTrajectory trajecto_state(move_group_interface_ptr->getCurrentState()->getRobotModel(), move_group_interface_ptr->getName());

    trajecto_state.setRobotTrajectoryMsg(*move_group_interface_ptr->getCurrentState(), trajectory);

    std::vector<std::vector<double>> trajecto_waypoint_joint;
    for (std::size_t i = 1; i < trajecto_state.getWayPointCount(); ++i)
    {
      std::vector<double> waypoint_joint;
      trajecto_state.getWayPointPtr(i)->copyJointGroupPositions(joint_model_group_ptr, waypoint_joint);
      trajecto_waypoint_joint.push_back(waypoint_joint);
    }


    return trajecto_waypoint_joint;



  }


  void trajecto_scan_to_bari( geometry_msgs::PoseStamped bari_pose){

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.3;

    ROS_INFO_NAMED("tutorial", "Go to bari position from scan");
    visual_tools_ptr->publishText(text_pose, "Go to bari position from scan", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    std::vector<std::vector<double>> traj6 = go_to_position( bari_pose);

  }


  void trajecto_scan_to_bari( std::vector<std::vector<geometry_msgs::PoseStamped>> vec_grasping_poses, int ID_grasp){

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.3;

    ROS_INFO_NAMED("tutorial", "Go to bari position from scan");
    visual_tools_ptr->publishText(text_pose, "Go to bari position from scan", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    std::vector<std::vector<double>> traj0 = go_to_position(vec_grasping_poses[0], ID_grasp);

    std::vector<std::vector<double>> traj1;
    if (ID_grasp != -1){
      traj1 = trajecto_approch( vec_grasping_poses[1][ID_grasp]);
    }
    traj0.insert(traj0.end(),std::make_move_iterator(traj1.begin()),std::make_move_iterator(traj1.end()));



    export_trajectory(traj0);

  }


  void trajecto_initial_to_scan_and_bari(std::vector<moveit_msgs::CollisionObject> &collision_object_baris, geometry_msgs::PoseStamped scan_pose, std::vector<geometry_msgs::PoseStamped> &bari_poses){


    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.3;

    move_group_interface_ptr->setStartState(*move_group_interface_ptr->getCurrentState());


    std::vector<std::vector<double>> traj1 = go_to_position(scan_pose);

    visual_tools_ptr->publishText(text_pose, "Go to scan position from origin", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



    load_bari_in_scene_simulation(collision_object_baris);
    load_carton_in_scene(collision_object_baris);


    std::chrono::seconds dura70(70);

    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
    //ROS_INFO_NAMED("tutorial", "Add an object into the world");
    //planning_scene_interface.addCollisionObjects(collision_objects);

    // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
    visual_tools_ptr->publishText(text_pose, "Add object", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();


    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");




    visual_tools_ptr->deleteAllMarkers();
    visual_tools_ptr->publishText(text_pose, " current : show bari position", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    //visual_tools.publishAxisLabeled(bari_poses[0], "bari_pose");
    visual_tools_ptr->trigger(); // to apply changes








    move_group_interface_ptr->setStartState(*move_group_interface_ptr->getCurrentState());


    trajecto_scan_to_bari( bari_poses[0]);

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


    //trajecto_approch(move_group_interface, collision_object_baris, planning_scene_interface, visual_tools, bari_poses[0]);


  }



  std::vector<std::vector<double>> trajecto_bari_to_scan( geometry_msgs::PoseStamped scan_pose){

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.3;

    ROS_INFO_NAMED("tutorial", "Go to scan position from bari");
    visual_tools_ptr->publishText(text_pose, "Go to scan position from bari", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    std::vector<std::vector<double>> traj6 = go_to_position(scan_pose);

    return traj6;

  }


  std::vector<std::vector<double>> trajecto_scan_to_out( geometry_msgs::PoseStamped &final_pose){

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.3;

    ROS_INFO_NAMED("tutorial", "Go to out position from scan");
    visual_tools_ptr->publishText(text_pose, "Go to out position from scan", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    std::vector<std::vector<double>> traj6 = go_to_position(final_pose);

    export_trajectory(traj6);

    return traj6;

  }



  std::vector<std::vector<double>> trajecto_bari_to_scan( geometry_msgs::PoseStamped scan_pose, std::vector<std::vector<geometry_msgs::PoseStamped>> &vec_grasping_poses, int ID_grasp){

    std::vector<std::vector<double>> traj0;
    std::vector<std::vector<double>> traj1;

    if (ID_grasp != -1){
      traj0 = trajecto_approch( vec_grasping_poses[0][ID_grasp]);
    }

    //std::cout<<"size traj"<<traj0.size()<<std::endl;

    traj1 = trajecto_bari_to_scan(scan_pose);

    traj0.insert(traj0.end(),std::make_move_iterator(traj1.begin()),std::make_move_iterator(traj1.end()));

    //std::cout<<"size traj"<<traj0.size()<<std::endl;

    export_trajectory(traj0);

    return traj0;

  }



  void trajecto_out_to_bari(geometry_msgs::PoseStamped &bari_pose){

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.3;

    ROS_INFO_NAMED("tutorial", "Go to bari position from out"); 
    visual_tools_ptr->publishText(text_pose, "Go to bari position from out", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    std::vector<std::vector<double>> traj5 = go_to_position(bari_pose);


  }

  void trajecto_out_to_bari(std::vector<std::vector<geometry_msgs::PoseStamped>> &vec_grasping_poses, int ID_grasp){

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.3;

    ROS_INFO_NAMED("tutorial", "Go to bari position from out"); 
    visual_tools_ptr->publishText(text_pose, "Go to bari position from out", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    std::vector<std::vector<double>> traj5 = go_to_position(vec_grasping_poses[0], ID_grasp);

    if (ID_grasp != -1){
      std::vector<std::vector<double>> traj6 = trajecto_approch( vec_grasping_poses[1][ID_grasp]);
      traj5.insert(traj5.end(),std::make_move_iterator(traj6.begin()),std::make_move_iterator(traj6.end()));

      export_trajectory(traj5);
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



  void trajecto_bari_to_out(std::vector<std::vector<geometry_msgs::PoseStamped>> &vec_grasping_poses, int ID_grasp, geometry_msgs::PoseStamped &final_pose){

    if (ID_grasp != -1){
      trajecto_approch( vec_grasping_poses[0][ID_grasp]);
    }
    move_group_interface_ptr->setStartStateToCurrentState();

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    std::vector<std::vector<double>> traj6 = go_to_position( final_pose);

    export_trajectory(traj6);

  }


  void trajecto_bari_to_out( geometry_msgs::PoseStamped &final_pose){



    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.3;

    ROS_INFO_NAMED("tutorial", "Go to out position from bari"); 
    visual_tools_ptr->publishText(text_pose, "Go to out position from bari", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();


    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    std::vector<std::vector<double>> traj6 = go_to_position(final_pose);

  }

  std::vector<std::vector<geometry_msgs::PoseStamped>> load_grasping_pose(float distance_approch, std::string FilePath){


    std::vector<std::vector<geometry_msgs::PoseStamped>> vec_grasping_poses;
    std::vector<geometry_msgs::PoseStamped> grasping_poses;
    std::vector<geometry_msgs::PoseStamped> grasping_poses_true;

    //TODO CHANGE
    std::ifstream fichier(FilePath);
    //std::ifstream fichier("/home/guillaume/Téléchargements/somfyBarrel.txt");
    //std::ifstream fichier("/home/guillaume/Téléchargements/5067976barillet005_SCHUNK-0340012MPG40ouvertv301centeredGripper_v1.5_a2_o14_r8_x20_y20_c0.001_success.txt");
    
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
          geometry_msgs::PoseStamped tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, distance_approch);
          grasping_poses.push_back(tf_transformed);
          tf_transformed = link6_in_bari_grasp(tf_tcp_in_bari, 0.00);
          grasping_poses_true.push_back(tf_transformed);

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

    return vec_grasping_poses;
  } 


  void full_scenario_grasp_robot(int M, int N){

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.3;


    std::string FilePath = "/home/guillaume/Téléchargements/somfyBarrel.txt";
    std::vector<std::vector<geometry_msgs::PoseStamped>> vec_grasping_poses = load_grasping_pose(0.05, FilePath);


    std::vector<moveit_msgs::CollisionObject> collision_object_baris;



    move_group_interface_ptr->setStartState(*move_group_interface_ptr->getCurrentState());

    std::vector<double> vec_joint; 
    geometry_msgs::PoseStamped final_poses;
    geometry_msgs::PoseStamped scan_pose;

    // moveit::core::RobotStatePtr current_state = move_group_interface_ptr->getCurrentState();
    // current_state->copyJointGroupPositions(joint_model_group_ptr, vec_joint);
    // std::cout << vec_joint.size() << std::endl;


    fill_vector_cin_angle(vec_joint, "please give the J_INIT pose in 1 line : j1 j2 j3 j4 j5 j6");



    std::vector<std::vector<double>> traj1 = go_to_position(vec_joint);



    scan_pose = get_pose("please give the SCAN pose in 1 line : tx ty tz rx ry rz"); 

    move_group_interface_ptr->setStartStateToCurrentState();


    int Nb_attempt = 3;
    double time_limit = 3.0;
    std::string planner_id = "LazyPRMstar";

    setup_planner(Nb_attempt, time_limit, planner_id);



    std::vector<std::vector<double>> traj2 = go_to_position(scan_pose);

    export_trajectory(traj2);

    visual_tools_ptr->publishText(text_pose, "Go to scan position from origin", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");




    //load_bari_in_scene(planning_scene_interface, move_group_interface, collision_object_baris, center);
    //load_carton_in_scene(visual_tools, planning_scene_interface, move_group_interface, collision_object_baris, center);


    std::vector<double> boxInsideSizeM;// = {0.269572 0.184798 0.177178};
    std::vector<double> boxInsidePoseM;// = {0.0464935, -0.823521, -0.275351, 337.809, 2.13806, 59.0498};
    std::vector<double> boxThickSideBottomM;// = {0.006, 0.016};

    fill_vector_cin(boxInsideSizeM, "please give the size of the inside of the box in 1 line : tx ty tz");
    fill_vector_cin(boxInsidePoseM, "please give the POSE of the inside of the box in 1 line : tx ty tz rx ry rz");
    fill_vector_cin(boxThickSideBottomM, "please give the size of the wall of the box in 1 line : tx ty");


    create_carton_in_scene(collision_object_baris, boxInsideSizeM, boxInsidePoseM, boxThickSideBottomM );



    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
    ROS_INFO_NAMED("tutorial", "HELLLLO");
    //planning_scene_interface.addCollisionObjects(collision_objects);

    // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
    visual_tools_ptr->publishText(text_pose, "Add object", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();


    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");




    visual_tools_ptr->deleteAllMarkers();
    visual_tools_ptr->publishText(text_pose, " current : show bari position", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    //visual_tools.publishAxisLabeled(bari_poses[0], "bari_pose");
    visual_tools_ptr->trigger(); // to apply changes


    // TODO CHANGE
    //std::string modelpath = "/home/pc-m/Documents/My-cao/barillet.obj";
    std::string modelpath = "package://geometric_shapes/test/resources/5067976_barillet_005.obj";
    //std::string modelpath = "package://geometric_shapes/test/resources/rc2.obj";
    shape_msgs::Mesh mesh_bary = load_mesh(modelpath);


    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::PoseStamped pose = get_pose("please give the bari pose in 1 line : tx ty tz rx ry rz");
    std::string str_id_obj = "bari0";

    load_obj_in_scene( collision_object_baris, mesh_bary, pose, str_id_obj);

    int ID_grasp; //bari
    //int ID_grasp = 1;

    std::cout << "give the ID_grasp (-1 if you want to let moveit choose) : " ;
    std::cin >> ID_grasp ;
    std::cout << "ID_grasp " << ID_grasp << std::endl ;

    if (ID_grasp != -1){
      vec_grasping_poses[0][ID_grasp].header.frame_id = str_id_obj;
      vec_grasping_poses[1][ID_grasp].header.frame_id = str_id_obj;
    }
    else{
      for (int k=0; k<vec_grasping_poses[0].size(); k++){
        vec_grasping_poses[0][k].header.frame_id = str_id_obj;
        vec_grasping_poses[1][k].header.frame_id = str_id_obj;
      }
    }
    for (int k=0; k<vec_grasping_poses[0].size(); k++){
      vec_grasping_poses[0][k].header.frame_id = str_id_obj;
      vec_grasping_poses[1][k].header.frame_id = str_id_obj;
    }





    move_group_interface_ptr->setStartState(*move_group_interface_ptr->getCurrentState());


    trajecto_scan_to_bari( vec_grasping_poses, ID_grasp);



    for (int i = 0; i < M; i++){
      

      std::cout<< "id bari : " << collision_object_baris[i*(N+1)].id << std::endl;

      // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
      // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
      ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1));
      move_group_interface_ptr->attachObject("bari0", "link_tool");

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


      visual_tools_ptr->publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      visual_tools_ptr->trigger();

      move_group_interface_ptr->setStartStateToCurrentState();

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

      final_poses = get_pose("please give the DEPOSE pose in 1 line : tx ty tz rx ry rz");


      //trajecto_bari_to_out_by_scan(collision_object_baris, scan_pose, final_poses[0], vec_grasping_poses, ID_grasp);


      trajecto_bari_to_scan(scan_pose, vec_grasping_poses, ID_grasp);


      trajecto_scan_to_out(final_poses);



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
      move_group_interface_ptr->detachObject("bari0");

      //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


      // Show text in RViz of status
      visual_tools_ptr->deleteAllMarkers();
      visual_tools_ptr->publishText(text_pose, "Object detached from robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      visual_tools_ptr->trigger();


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

        move_group_interface_ptr->setStartState(*move_group_interface_ptr->getCurrentState());

        //modelpath = "package://geometric_shapes/test/resources/5067976_barillet_005.obj";
        //std::string modelpath = "package://geometric_shapes/test/resources/rc2.obj";
        // Define a pose for the box (specified relative to frame_id)
        pose = get_pose("please give the bari pose in 1 line : tx ty tz rx ry rz");
        str_id_obj = "bari0";

        load_obj_in_scene( collision_object_baris, mesh_bary, pose, str_id_obj);

        std::cout << "give the ID_grasp (-1 if you want to let moveit choose) : " ;
        std::cin >> ID_grasp ;
        std::cout << "ID_grasp " << ID_grasp << std::endl ;

        std::cout<< "id bari : " << collision_object_baris[i*(N+1) + j + 1].id << std::endl;
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

        trajecto_out_to_bari(vec_grasping_poses, ID_grasp);


        // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
        // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
        ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1) + j + 1);

        move_group_interface_ptr->attachObject("bari0", "link_tool");

        visual_tools_ptr->publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
        visual_tools_ptr->trigger();

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
        move_group_interface_ptr->setStartStateToCurrentState();

        final_poses = get_pose("please give the DEPOSE pose in 1 line : tx ty tz rx ry rz");


        trajecto_bari_to_out( vec_grasping_poses, ID_grasp, final_poses);


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
        move_group_interface_ptr->detachObject("bari0");





        //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");






        // Show text in RViz of status
        visual_tools_ptr->deleteAllMarkers();
        visual_tools_ptr->publishText(text_pose, "Object detached from robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
        visual_tools_ptr->trigger();


        /* Wait for MoveGroup to receive and process the attached collision object message */
        //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");



      }



      move_group_interface_ptr->setStartState(*move_group_interface_ptr->getCurrentState());


      std::cout<< "id bari : " << collision_object_baris[i*(N+1) + N + 1].id << std::endl;

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


      trajecto_out_to_bari(vec_grasping_poses, ID_grasp);

      // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
      // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
      ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1) + N + 1);

      move_group_interface_ptr->attachObject("bari0", "link_tool");

      visual_tools_ptr->publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      visual_tools_ptr->trigger();

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
    move_group_interface_ptr->setStartStateToCurrentState();

    final_poses = get_pose("please give the DEPOSE pose in 1 line : tx ty tz rx ry rz");


    trajecto_bari_to_out( vec_grasping_poses, ID_grasp, final_poses);


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
    move_group_interface_ptr->detachObject("bari0");

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Show text in RViz of status
    visual_tools_ptr->deleteAllMarkers();
    visual_tools_ptr->publishText(text_pose, "Object detached from robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();


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
    planning_scene_interface_ptr->removeCollisionObjects(object_ids);

    // Show text in RViz of status
    visual_tools_ptr->publishText(text_pose, "Objects removed", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();


  }


  void full_scenario_grasp( int M, int N){

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.3;

    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    // Adding objects to the environment
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // First let's plan to another simple goal with no objects in the way.

    geometry_msgs::PoseStamped scan_pose;
    scan_pose.header.frame_id = "base_link";

    scan_pose.pose.orientation.x = -0.29956;
    scan_pose.pose.orientation.y = 0.65046;
    scan_pose.pose.orientation.z = -0.25386;
    scan_pose.pose.orientation.w = 0.65018;
    scan_pose.pose.position.x = -0.089795;
    scan_pose.pose.position.y = -0.71612;
    scan_pose.pose.position.z = 0.3414;

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


          final_poses.push_back(target_pose_final);


        }
      }
    }

    std::string FilePath = "/home/guillaume/Téléchargements/somfyBarrel.txt";
    std::cout << "grasping file loaded : " << FilePath << std::endl;

    std::vector<std::vector<geometry_msgs::PoseStamped>> vec_grasping_poses = load_grasping_pose(0.05, FilePath);

    std::vector<moveit_msgs::CollisionObject> collision_object_baris;


    move_group_interface_ptr->setStartState(*move_group_interface_ptr->getCurrentState());

    std::vector<double> vec_joint(6); 
    vec_joint[0] = -90*M_PI/180.0;
    vec_joint[1] = 19*M_PI/180.0;
    vec_joint[2] = 104*M_PI/180.0;
    vec_joint[3] = -2*M_PI/180.0;
    vec_joint[4] = 58*M_PI/180.0;
    vec_joint[5] = -134*M_PI/180.0;
    // vec_joint.push_back(0);
    // vec_joint.push_back(0);
    // vec_joint.push_back(0);
    // vec_joint.push_back(0);
    // vec_joint.push_back(0);


    std::vector<std::vector<double>> traj0 = go_to_position(vec_joint);


    // - SBLkConfigDefault
    // - ESTkConfigDefault
    // - LBKPIECEkConfigDefault
    // - BKPIECEkConfigDefault
    // - KPIECEkConfigDefault
    // - RRTkConfigDefault
    // - RRTConnectkConfigDefault
    // - RRTstarkConfigDefault
    // - TRRTkConfigDefault
    // - PRMkConfigDefault
    // - PRMstarkConfigDefault
    // - FMTkConfigDefault
    // - BFMTkConfigDefault
    // - PDSTkConfigDefault
    // - STRIDEkConfigDefault
    // - BiTRRTkConfigDefault
    // - LBTRRTkConfigDefault
    // - BiESTkConfigDefault
    // - ProjESTkConfigDefault
    // - LazyPRMkConfigDefault
    // - LazyPRMstarkConfigDefault
    // - SPARSkConfigDefault
    // - SPARStwokConfigDefault
    // - TrajOptDefault

    //TODO TEST
    int Nb_attempt = 1;
    double time_limit = 2;
    //std::string planner_id = "AnytimePathShortening";
    std::string planner_id = "SBLkConfigDefault";

  

    setup_planner(Nb_attempt, time_limit, planner_id);

    move_group_interface_ptr->setStartState(*move_group_interface_ptr->getCurrentState());

    //std::vector<std::vector<double>> traj1 = go_to_position(scan_pose);


    //visual_tools_ptr->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo 9");

    load_bari_in_scene_simulation(collision_object_baris);
    load_carton_in_scene(collision_object_baris);

    //TODO CHANGE
    int ID_grasp = 13; //bari
    //int ID_grasp = 16; //bari
    //int ID_grasp = 1;
    //visual_tools_ptr->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo 5");

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


    move_group_interface_ptr->setStartState(*move_group_interface_ptr->getCurrentState());
    trajecto_scan_to_bari( vec_grasping_poses, ID_grasp);

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");




    for (int i = 0; i < M; i++){
      

      std::cout<< "id bari : " << collision_object_baris[i*(N+1)].id << std::endl;

      // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
      // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
      ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1));
      move_group_interface_ptr->attachObject(collision_object_baris[i*(N+1)].id, "link_tool");

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


      visual_tools_ptr->publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      visual_tools_ptr->trigger();

      move_group_interface_ptr->setStartStateToCurrentState();

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

      //trajecto_bari_to_out_by_scan(collision_object_baris, scan_pose, final_poses[i*(N+1)], vec_grasping_poses, ID_grasp);



      trajecto_bari_to_scan(scan_pose, vec_grasping_poses, ID_grasp);


      trajecto_scan_to_out(final_poses[i*(N+1)]);



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
      move_group_interface_ptr->detachObject(collision_object_baris[i*(N+1)].id);

      //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


      // Show text in RViz of status
      visual_tools_ptr->deleteAllMarkers();
      visual_tools_ptr->publishText(text_pose, "Object detached from robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      visual_tools_ptr->trigger();


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

        move_group_interface_ptr->setStartState(*move_group_interface_ptr->getCurrentState());


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

        trajecto_out_to_bari(vec_grasping_poses, ID_grasp);


        // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
        // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
        ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1) + j + 1);

        move_group_interface_ptr->attachObject(collision_object_baris[i*(N+1) + j + 1].id, "link_tool");

        visual_tools_ptr->publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
        visual_tools_ptr->trigger();

        //std::chrono::seconds dura10(10);


        move_group_interface_ptr->setStartStateToCurrentState();





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
        move_group_interface_ptr->setStartStateToCurrentState();

        trajecto_bari_to_out( vec_grasping_poses, ID_grasp, final_poses[i*(N+1) + j + 1]);


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
        move_group_interface_ptr->detachObject(collision_object_baris[i*(N+1) + j + 1].id);





        //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");






        // Show text in RViz of status
        visual_tools_ptr->deleteAllMarkers();
        visual_tools_ptr->publishText(text_pose, "Object detached from robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
        visual_tools_ptr->trigger();


        /* Wait for MoveGroup to receive and process the attached collision object message */
        //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");



      }



      move_group_interface_ptr->setStartState(*move_group_interface_ptr->getCurrentState());


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


      trajecto_out_to_bari(vec_grasping_poses, ID_grasp);

      // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
      // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
      ROS_INFO_NAMED("tutorial", "Attach the object to the robot: %d", i*(N+1) + N + 1);

      move_group_interface_ptr->attachObject(collision_object_baris[i*(N+1) + N + 1].id, "link_tool");

      visual_tools_ptr->publishText(text_pose, "Object attached to robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
      visual_tools_ptr->trigger();

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
    move_group_interface_ptr->setStartStateToCurrentState();


    trajecto_bari_to_out( vec_grasping_poses, ID_grasp, final_poses[(M-1)*(N+1) + N + 1]);


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
    move_group_interface_ptr->detachObject(collision_object_baris[(M-1)*(N+1) + N + 1].id);

    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Show text in RViz of status
    visual_tools_ptr->deleteAllMarkers();
    visual_tools_ptr->publishText(text_pose, "Object detached from robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();


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
    planning_scene_interface_ptr->removeCollisionObjects(object_ids);

    // Show text in RViz of status
    visual_tools_ptr->publishText(text_pose, "Objects removed", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_ptr->trigger();


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





};


// void run_ros(int argc, char** argv)
// {

//   std::cout << "HI THERE 0" << std::endl;
//   ros::init(argc, argv, "move_group_interface_tutorial");
//   ros::NodeHandle node_handle;

//   // ROS spinning must be running for the MoveGroupInterface to get information
//   // about the robot's state. One way to do this is to start an AsyncSpinner
//   // beforehand.
//   ros::AsyncSpinner spinner(1);
//   spinner.start();

//   int i(0);
//   while(true)
//   {
//     std::cout << "thread --> " << ++i << std::endl;
//   }

// }
Moveit_Engine::Moveit_Engine()
{

}
Moveit_Engine::Moveit_Engine(int argc, char** argv)
{

  //std::thread first(run_ros, argc, argv);
  //std::cout << "thread launched" << std::endl;

  //first.join();
  //std::this_thread::sleep_for(std::chrono::seconds(2));
  //std::cout << "thread finished" << std::endl;

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  //static const std::string PLANNING_GROUP = "arm_group";

  // // The :planning_interface:`MoveGroupInterface` class can be easily
  // // setup using just the name of the planning group you would like to control and plan for.
  //moveit::planning_interface::MoveGroupInterface move_group_interface = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  //move_group_interfaceptr = &move_group_interface;

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  //planning_scene_interfaceptr = &planning_scene_interface;


  //Raw pointers are frequently used to refer to the planning group for improved performance.
  //joint_model_group = move_group_interfaceptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  std::cout << "HI THERE 1" << std::endl;
}

Moveit_Engine::Moveit_Engine(int argc, char** argv, moveit::planning_interface::MoveGroupInterface move_group_interface, moveit::planning_interface::PlanningSceneInterface planning_scene_interface, const moveit::core::JointModelGroup* joint_model_group)
{

  //std::thread first(run_ros, argc, argv);
  //std::cout << "thread launched" << std::endl;

  //first.join();
  //std::this_thread::sleep_for(std::chrono::seconds(2));
  //std::cout << "thread finished" << std::endl;

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  //static const std::string PLANNING_GROUP = "arm_group";

  // // The :planning_interface:`MoveGroupInterface` class can be easily
  // // setup using just the name of the planning group you would like to control and plan for.
  //moveit::planning_interface::MoveGroupInterface move_group_interface = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  move_group_interface_ptr = &move_group_interface;

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface_ptr = &planning_scene_interface;


  //Raw pointers are frequently used to refer to the planning group for improved performance.
  joint_model_group_ptr = move_group_interface_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

}
Moveit_Engine::~Moveit_Engine()
{
}

const std::string Moveit_Engine::PLANNING_GROUP = "arm_group";


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  static const std::string PLANNING_GROUP = "arm_group";


  //robot_model_loader::RobotModelLoader robot_model_loader("arm_group");
  //const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());


  // // The :planning_interface:`MoveGroupInterface` class can be easily
  // // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

  //std::thread first(run_ros, argc, argv);
  //std::cout << "thread launched" << std::endl;

  //first.join();
  //std::this_thread::sleep_for(std::chrono::seconds(2));
  //std::cout << "thread finished" << std::endl;

  //Moveit_Engine moveit_engine_test = Moveit_Engine(argc,argv,move_group_interface,planning_scene_interface,joint_model_group);
  Moveit_Engine moveit_engine_test = Moveit_Engine(argc,argv);

  moveit_engine_test.move_group_interface_ptr = &move_group_interface;
  moveit_engine_test.planning_scene_interface_ptr = &planning_scene_interface;
  moveit_engine_test.joint_model_group_ptr = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  moveit_engine_test.visual_tools_ptr = &visual_tools;

  // std::cout << "HI THERE 5098" << std::endl;

  // std::cout << "HI THERE 111" << moveit_engine_test.move_group_int;erfaceptr << std::endl;

  // moveit::core::RobotStatePtr state = move_group_interface.getCurrentState();

  // std::cout << "HI THERE 112"  << state << std::endl;

  // move_group_interface.setStartState(*state);

  //std::cout << "HI THERE 113" << std::endl;
  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  //static const std::string PLANNING_GROUP = "arm_group";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  //moveit::planning_interface::MoveGroupInterface move_group_interface(moveit_engine_test.PLANNING_GROUP);

  // // We will use the :planning_interface:`PlanningSceneInterface`
  // // class to add and remove collision objects in our "virtual world" scene
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  // Raw pointers are frequently used to refer to the planning group for improved performance.
  // const moveit::core::JointModelGroup* joint_model_group =
  //    moveit_engine_test.move_group_interfaceptr->getCurrentState()->getJointModelGroup(moveit_engine_test.PLANNING_GROUP);


  // Visualization
  // ^^^^^^^^^^^^^
  //

  visual_tools.deleteAllMarkers();


  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Demo MoveIt", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();



  int M = 2;
  int N = 2; //(+1 bari detecté par scan)


  //full_scenario_without_attach( move_group_interface, collision_object_baris, planning_scene_interface, scan_pose, visual_tools, joint_model_group, final_poses, bari_poses, M, N);

  //moveit_engine_test.full_scenario( collision_object_baris, scan_pose, final_poses, bari_poses, grasping_poses, M, N);

  //moveit_engine_test.full_scenario_grasp(collision_object_baris, scan_pose, final_poses, bari_poses, vec_grasping_poses, M, N);
  moveit_engine_test.full_scenario_grasp( M, N);

  //moveit_engine_test.full_scenario_grasp_robot(M, N);



  //full_scenario_pick_place( move_group_interface, collision_object_baris, planning_scene_interface, scan_pose, visual_tools, joint_model_group, final_poses, bari_poses, M, N);


  //scenario_test_scan_attach( move_group_interface, collision_object_baris, planning_scene_interface, scan_pose, visual_tools, joint_model_group, final_poses, bari_poses, M, N);


  //ok scenario_test_out_attach( move_group_interface, collision_object_baris, planning_scene_interface, scan_pose, visual_tools, joint_model_group, final_poses, bari_poses, M, N);




  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<<"Global time to find plan : "<<moveit_engine_test.Global_time_find_planning<<" s "<<std::endl;
  std::cout<<"Global time to execute traj : "<<moveit_engine_test.Global_time_traj<<" s "<<std::endl;
  std::cout<<"Global nb of step in the traj : "<<moveit_engine_test.Global_nb_step_traj<<std::endl;
  std::cout<<"Global move of the tool : "<<moveit_engine_test.Global_move_tool<< " m"<<std::endl;
  std::cout<<"Global angle move of the tool : "<<moveit_engine_test.Global_angle_move_tool<< " m"<<std::endl;
  std::cout<<"Nombre de fail : "<< moveit_engine_test.NB_fail<< std::endl;
  std::cout<< "###########" << std::endl;
  std::cout<< "###########" << std::endl;



  // END_TUTORIAL

  ros::shutdown();

  return 0;
}
