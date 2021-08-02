#pragma once

#include "MoveIt_Engine.h"

#include <extcode.h>

//==============================================================================
//   Declarations
//==============================================================================


////! Errors handle
//extern "C" __declspec(dllexport) const char* __cdecl GetError(const int errorCode);
//
//
////! Instance handle
extern "C" __declspec(dllexport) int32_t __cdecl Initialization();
// dllmain.cpp : Defines the entry point for the DLL application.


//==============================================================================
//   Definitions
//==============================================================================
//
//! Errors handle
extern "C" __declspec(dllexport) const char* __cdecl GetError(const int errorCode);

//! Instance handle
extern "C" __declspec(dllexport) int32_t __cdecl Initialization();

extern "C" __declspec(dllexport) int32_t __cdecl Release();


extern "C" __declspec(dllexport) int __cdecl SetLoggingPath(const LStrHandle MoveIt_log);


extern "C" __declspec(dllexport) int __cdecl do_simulation(const int N, const int M);

extern "C" __declspec(dllexport) int __cdecl go_to_position_joint(arr1Dd_LV joints, arr2Dd_LV traj);

extern "C" __declspec(dllexport) int __cdecl go_to_position_pose(Position Pose, Orientation Rota, LStrHandle repere,
                                                                arr2Dd_LV traj);
extern "C" __declspec(dllexport) int __cdecl go_to_position_grasp(int ID_grasp, bool true_or_approch, LStrHandle repere, arr2Dd_LV traj);

extern "C" __declspec(dllexport) int __cdecl go_to_position_cartesian(Position Pose, Orientation Rota, LStrHandle repere, arr2Dd_LV traj);

extern "C" __declspec(dllexport) int __cdecl GetStatistics(double* time_find_planning, double* time_traj,
                                                           double* nb_step_traj, double* distance_traveled_tool,
                                                           double* angle_traveled_tool, double* nb_fail);


