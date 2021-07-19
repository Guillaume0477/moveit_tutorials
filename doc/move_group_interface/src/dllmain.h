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
//
//
//extern "C" __declspec(dllexport) int32_t __cdecl Release();
//
//
////! Logs handle
//extern "C" __declspec(dllexport) int __cdecl SetLoggingPath(const LStrHandle InnoViA_log);
//
//
////! Main function
//extern "C" __declspec(dllexport) int __cdecl CreateBox_cao(const Dims dimensions, const Position pose, const Orientation rot, LStrHandle path);
//
//
//extern "C" __declspec(dllexport) int __cdecl CreateBox(const Dims boxInsideSizeM_arg, const Position boxThickSideBottomM_arg, const Position boxInsidePoseMpos_arg, const Orientation boxInsidePoseMrot_arg, const float bigger_wall_arg);
//
//
//extern "C" __declspec(dllexport) int __cdecl Init_object(LStrHandle path, int size_obj);
//
//
//extern "C" __declspec(dllexport) int __cdecl DeleteAllActorsOut();
//
//
//extern "C" __declspec(dllexport) int __cdecl DeleteActor(const int ID_box, const int ID_Type, const int index);
//
//
//
//extern "C" __declspec(dllexport) int __cdecl FillBox(const int ID_box, const int ID_Type, const int Nb_object);
//
//extern "C" __declspec(dllexport) int __cdecl FillBox2(const int ID_box, const int ID_Type, const int Nb_object);
//
//
//extern "C" __declspec(dllexport) int __cdecl DoAllStep();
//
//extern "C" __declspec(dllexport) unsigned int __cdecl DoStep();
//
//extern "C" __declspec(dllexport) unsigned int __cdecl DoStep_N(uint32_t Nb_step);
//
extern "C" __declspec(dllexport) int __cdecl GetAllItemsPosition(int ID_box, int ID_Type, arr1Df_LV Tx, arr1Df_LV Ty, arr1Df_LV Tz, arr1Df_LV q0, arr1Df_LV q1, arr1Df_LV q2, arr1Df_LV q3, arr1Df_LV is_valid);
//
//extern "C" __declspec(dllexport) int __cdecl Shuffle(int ID_box, float force_value);


