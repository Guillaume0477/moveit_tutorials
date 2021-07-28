// dllmain.cpp : Defines the entry point for the DLL application.


#include "dllmain.h"

#include "ErrorCodes.h"

#include <string>
//==============================================================================
//   Definitions
//==============================================================================
//
//! Errors handle
__declspec(dllexport) const char* __cdecl GetError(const int errorCode) {
	std::string str = GetErrorFromCode(errorCode);
	const size_t size = str.size() + 1;
	char* ret = new (char[size]);
	memcpy(ret, str.c_str(), str.size() + 1);
	return ret;
}

//! Instance handle
__declspec(dllexport) int32_t __cdecl Initialization()
{
	if (!MoveIt_Engine::CheckInstance()) 
		MoveIt_Engine::CreateInstance();



	int argc = 0;
	char** argv = nullptr; 

	std::cout << "in Initialisation" << std::endl;

	std::cout << " ros::o=k()" << ros::ok() << std::endl;
	std::system("start roslaunch staubli_moveit_config demo.launch rviz_tutorial:=true;");

	std::this_thread::sleep_for(std::chrono::seconds(30));

	// std::cout << argv[0] << argv[1] << argv[2] << argv[3] << argv[4] << std::endl;
	ros::init(argc, argv, "move_group_interface_tutorial");

	std::cout << "in Initialisation" << std::endl;

	std::cout << " ros::ok()" << ros::ok() << std::endl;
	ros::NodeHandle node_handle;
	// ROS spinning must be running for the MoveGroupInterface to get information
	// about the robot's state. One way to do this is to start an AsyncSpinner
	// beforehand.
	ros::AsyncSpinner spinner(1);
	spinner.start();
	std::cout << "in Initialisation" << std::endl;

	std::cout << " ros::ok()" << ros::ok() << std::endl;
	static const std::string PLANNING_GROUP = "arm_group";
	// robot_model_loader::RobotModelLoader robot_model_loader("arm_group");
	// const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
	// ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	// // The :planning_interface:`MoveGroupInterface` class can be easily
	// // setup using just the name of the planning group you would like to control and plan for.
	moveit::planning_interface::MoveGroupInterface move_group_interface = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

	// We will use the :planning_interface:`PlanningSceneInterface`
	// class to add and remove collision objects in our "virtual world" scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
	// and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

	// std::thread first(run_ros, argc, argv);
	// std::cout << "thread launched" << std::endl;

	// first.join();
	// std::this_thread::sleep_for(std::chrono::seconds(2));
	// std::cout << "thread finished" << std::endl;
	std::cout << "in Initialisation" << std::endl;

	std::cout << " ros::ok()" << ros::ok() << std::endl;
	// Moveit_Engine moveit_engine_test = Moveit_Engine(argc,argv,move_group_interface,planning_scene_interface,joint_model_group);
	
	MoveIt_Engine::GetInstance().move_group_interface_ptr = &move_group_interface;
    MoveIt_Engine::GetInstance().planning_scene_interface_ptr = &planning_scene_interface;
    MoveIt_Engine::GetInstance().joint_model_group_ptr = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    MoveIt_Engine::GetInstance().visual_tools_ptr = &visual_tools;

    std::cout << "###########" << std::endl;
    std::cout << "###########" << std::endl;
    std::cout << "Global time to find plan : " << MoveIt_Engine::GetInstance().Global_time_find_planning << " s "
              << std::endl;
    std::cout << "Global time to execute traj : " << MoveIt_Engine::GetInstance().Global_time_traj << " s "
              << std::endl;
    std::cout << "Global nb of step in the traj : " << MoveIt_Engine::GetInstance().Global_nb_step_traj << std::endl;
    std::cout << "Global move of the tool : " << MoveIt_Engine::GetInstance().Global_move_tool << " m" << std::endl;
    std::cout << "Global angle move of the tool : " << MoveIt_Engine::GetInstance().Global_angle_move_tool << std::endl;
    std::cout << "Nombre de fail : " << MoveIt_Engine::GetInstance().NB_fail << std::endl;
    std::cout << "###########" << std::endl;
    std::cout << "###########" << std::endl;

    MoveIt_Engine::GetInstance().full_scenario_grasp(2, 2, "LazyPRMstar");

    std::cout << "Global time to find plan : " << MoveIt_Engine::GetInstance().Global_time_find_planning << " s "
              << std::endl;
    std::cout << "Global time to execute traj : " << MoveIt_Engine::GetInstance().Global_time_traj << " s "
              << std::endl;
    std::cout << "Global nb of step in the traj : " << MoveIt_Engine::GetInstance().Global_nb_step_traj << std::endl;
    std::cout << "Global move of the tool : " << MoveIt_Engine::GetInstance().Global_move_tool << " m" << std::endl;
    std::cout << "Global angle move of the tool : " << MoveIt_Engine::GetInstance().Global_angle_move_tool << std::endl;
    std::cout << "Nombre de fail : " << MoveIt_Engine::GetInstance().NB_fail << std::endl;

	return 0;
}

__declspec(dllexport) int32_t __cdecl Release() {

	if (MoveIt_Engine::CheckInstance())
	{
		ros::shutdown();
		MoveIt_Engine::ReleaseInstance();
	}
	return 0;
}

//! Logs handle
__declspec(dllexport) int __cdecl SetLoggingPath(const LStrHandle MoveIt_log) {
	if (!MoveIt_Engine::CheckInstance()) {
		MoveIt_Engine::CreateInstance();
		return MoveIt_Engine::GetInstance().SetLoggingPath(MoveIt_log);
	}
	else
		return MoveIt_Engine::GetInstance().SetLoggingPath(MoveIt_log);
}


__declspec(dllexport) int __cdecl go_to_position_joint(arr1Dd_LV joints, arr2Dd_LV traj)
 {
  if (!MoveIt_Engine::CheckInstance())
    MoveIt_Engine::CreateInstance();

  int argc = 0;
  char** argv = nullptr;

  std::cout << "in Initialisation" << std::endl;

  std::cout << " ros::o=k()" << ros::ok() << std::endl;
  std::system("start roslaunch staubli_moveit_config demo.launch rviz_tutorial:=true;");

  std::this_thread::sleep_for(std::chrono::seconds(30));

  // std::cout << argv[0] << argv[1] << argv[2] << argv[3] << argv[4] << std::endl;
  ros::init(argc, argv, "move_group_interface_tutorial");

  std::cout << "in Initialisation" << std::endl;

  std::cout << " ros::ok()" << ros::ok() << std::endl;
  ros::NodeHandle node_handle;
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::cout << "in Initialisation" << std::endl;

  std::cout << " ros::ok()" << ros::ok() << std::endl;
  static const std::string PLANNING_GROUP = "arm_group";
  // robot_model_loader::RobotModelLoader robot_model_loader("arm_group");
  // const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  // ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // // The :planning_interface:`MoveGroupInterface` class can be easily
  // // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface =
      moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

  // std::thread first(run_ros, argc, argv);
  // std::cout << "thread launched" << std::endl;

  // first.join();
  // std::this_thread::sleep_for(std::chrono::seconds(2));
  // std::cout << "thread finished" << std::endl;
  std::cout << "in Initialisation" << std::endl;

  std::cout << " ros::ok()" << ros::ok() << std::endl;
  // Moveit_Engine moveit_engine_test = Moveit_Engine(argc,argv,move_group_interface,planning_scene_interface,joint_model_group);

  MoveIt_Engine::GetInstance().move_group_interface_ptr = &move_group_interface;
  MoveIt_Engine::GetInstance().planning_scene_interface_ptr = &planning_scene_interface;
  MoveIt_Engine::GetInstance().joint_model_group_ptr =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  MoveIt_Engine::GetInstance().visual_tools_ptr = &visual_tools;

  std::cout << "###########" << std::endl;
  std::cout << "###########" << std::endl;
  std::cout << "Global time to find plan : " << MoveIt_Engine::GetInstance().Global_time_find_planning << " s "
            << std::endl;
  std::cout << "Global time to execute traj : " << MoveIt_Engine::GetInstance().Global_time_traj << " s " << std::endl;
  std::cout << "Global nb of step in the traj : " << MoveIt_Engine::GetInstance().Global_nb_step_traj << std::endl;
  std::cout << "Global move of the tool : " << MoveIt_Engine::GetInstance().Global_move_tool << " m" << std::endl;
  std::cout << "Global angle move of the tool : " << MoveIt_Engine::GetInstance().Global_angle_move_tool << std::endl;
  std::cout << "Nombre de fail : " << MoveIt_Engine::GetInstance().NB_fail << std::endl;
  std::cout << "###########" << std::endl;
  std::cout << "###########" << std::endl;


  if (!MoveIt_Engine::CheckInstance())
    return ERR_CLASS_INSTANCE;



  std::vector<double> join_goal;

  for (size_t i = 0; i < (*joints)->dimSize; i++) {
    join_goal.push_back((*joints)->elt[i]);
    std::cout << "LV target : " << join_goal[i] << std::endl;
  }
  MoveIt_Engine::GetInstance().setup_planner(3,3.0,"");



  MoveIt_Engine::GetInstance().move_group_interface_ptr->setStartState(*MoveIt_Engine::GetInstance().move_group_interface_ptr->getCurrentState());

  
  std::vector<std::vector<double>> trajectory = MoveIt_Engine::GetInstance().go_to_position(join_goal);
  
  std::cout << "trajectory size : " << trajectory.size() << " - " << trajectory[0].size() << std::endl;

  MgErr err(NumericArrayResize(fD, 2, (UHandle*)(&traj), trajectory.size() * trajectory[0].size() * sizeof(double)));

  (*traj)->dimSize[0] = (uint32_t)(trajectory.size());
  (*traj)->dimSize[1] = (uint32_t)(trajectory[0].size());

  for (size_t i = 0; i < trajectory.size(); i++)
  {
    for (size_t j = 0; j < trajectory[0].size(); j++) {
      (*traj)->elt[trajectory[0].size() * i + j] = trajectory[i][j];
	}
  }

  return 0;
}



__declspec(dllexport) int __cdecl go_to_position_pose(Position Pose, Orientation Rota, LStrHandle repere, arr2Dd_LV traj)
  {
  if (!MoveIt_Engine::CheckInstance())
    return ERR_CLASS_INSTANCE;


  std::string repere_str((char*)LStrBuf(*repere), 0, LStrLen(*repere));
  
  std::vector<double> vector_pose;
  vector_pose[0] = Pose.Tx;
  vector_pose[1] = Pose.Ty;
  vector_pose[2] = Pose.Tz;

  vector_pose[3] = Rota.Rx;
  vector_pose[4] = Rota.Ry;
  vector_pose[5] = Rota.Rz;

  geometry_msgs::PoseStamped Pose_goal = MoveIt_Engine::GetInstance().get_pose(vector_pose, repere_str);

  std::vector<std::vector<double>> trajectory = MoveIt_Engine::GetInstance().go_to_position(Pose_goal);

  std::cout << "trajectory size : " << trajectory.size() << " - " << trajectory[0].size() << std::endl;

  MgErr err(NumericArrayResize(fD, 2, (UHandle*)(&traj), trajectory.size() * trajectory[0].size() * sizeof(double)));

  (*traj)->dimSize[0] = (uint32_t)(trajectory.size());
  (*traj)->dimSize[1] = (uint32_t)(trajectory[0].size());

  for (size_t i = 0; i < trajectory.size(); i++)
  {
    for (size_t j = 0; j < trajectory[0].size(); j++)
    {
      (*traj)->elt[trajectory[0].size() * i + j] = trajectory[i][j];
    }
  }

  return 0;
}

__declspec(dllexport) int __cdecl do_simulation(const int N, const int M) {
  if (!MoveIt_Engine::CheckInstance())
		return ERR_CLASS_INSTANCE;

	MoveIt_Engine::GetInstance().full_scenario_grasp(N, M, "LazyPRMstar");
	std::cout << "Global time to find plan : " << MoveIt_Engine::GetInstance().Global_time_find_planning << " s "
			<< std::endl;
	std::cout << "Global time to execute traj : " << MoveIt_Engine::GetInstance().Global_time_traj << " s " << std::endl;
	std::cout << "Global nb of step in the traj : " << MoveIt_Engine::GetInstance().Global_nb_step_traj << std::endl;
	std::cout << "Global move of the tool : " << MoveIt_Engine::GetInstance().Global_move_tool << " m" << std::endl;
	std::cout << "Global angle move of the tool : " << MoveIt_Engine::GetInstance().Global_angle_move_tool << std::endl;
	std::cout << "Nombre de fail : " << MoveIt_Engine::GetInstance().NB_fail << std::endl;

}


__declspec(dllexport) int __cdecl GetStatistics(double* time_find_planning, double* time_traj, double* nb_step_traj,
                                                double* distance_traveled_tool, double* angle_traveled_tool, double* nb_fail)
{
	if (!MoveIt_Engine::CheckInstance())
		return ERR_CLASS_INSTANCE;

	*time_find_planning = MoveIt_Engine::GetInstance().Global_time_find_planning;
    *time_traj = MoveIt_Engine::GetInstance().Global_time_traj;
	*nb_step_traj = MoveIt_Engine::GetInstance().Global_nb_step_traj;
	*distance_traveled_tool = MoveIt_Engine::GetInstance().Global_move_tool;
	*angle_traveled_tool = MoveIt_Engine::GetInstance().Global_angle_move_tool;
	*nb_fail = MoveIt_Engine::GetInstance().NB_fail;


	std::cout << "Global time to find plan : " << MoveIt_Engine::GetInstance().Global_time_find_planning << " s " << std::endl;
	std::cout << "Global time to execute traj : " << MoveIt_Engine::GetInstance().Global_time_traj << " s " << std::endl;
	std::cout << "Global nb of step in the traj : " << MoveIt_Engine::GetInstance().Global_nb_step_traj << std::endl;
	std::cout << "Global move of the tool : " << MoveIt_Engine::GetInstance().Global_move_tool << " m" << std::endl;
	std::cout << "Global angle move of the tool : " << MoveIt_Engine::GetInstance().Global_angle_move_tool << std::endl;
	std::cout << "Nombre de fail : " << MoveIt_Engine::GetInstance().NB_fail << std::endl;
	return 0;
}



