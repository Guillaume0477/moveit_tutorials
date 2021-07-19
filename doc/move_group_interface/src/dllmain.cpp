// dllmain.cpp : Defines the entry point for the DLL application.


#include "dllmain.h"

#include "ErrorCodes.h"

#include <string>
//==============================================================================
//   Definitions
//==============================================================================
//
////! Errors handle
//__declspec(dllexport) const char* __cdecl GetError(const int errorCode) {
//	std::string str = GetErrorFromCode(errorCode);
//	const size_t size = str.size() + 1;
//	char* ret = new (char[size]);
//	memcpy(ret, str.c_str(), str.size() + 1);
//	return ret;
//}
//
//! Instance handle
//__declspec(dllexport) int32_t __cdecl Initialization()
//{
//	if (!MoveIt_Engine::CheckInstance()) 
//		MoveIt_Engine::CreateInstance();
//
//	MoveIt_Engine::GetInstance();
//	return 0;
//}
//
//__declspec(dllexport) int32_t __cdecl Release() {
//
//	if (PhysX_Engine::CheckInstance()) {
//		PhysX_Engine::GetInstance().cleanupPhysics(false);
//		PhysX_Engine::ReleaseInstance();
//	}
//	return 0;
//}
//
////! Logs handle
//__declspec(dllexport) int __cdecl SetLoggingPath(const LStrHandle InnoViA_log) {
//	if (!PhysX_Engine::CheckInstance()) {
//		PhysX_Engine::CreateInstance();
//		return PhysX_Engine::GetInstance().SetLoggingPath(InnoViA_log);
//	}
//	else
//		return PhysX_Engine::GetInstance().SetLoggingPath(InnoViA_log);
//}
//
////! Main function
//__declspec(dllexport) int __cdecl CreateBox_cao(const Dims dimensions, const Position pose, const Orientation rot, LStrHandle path) {
//	if (!PhysX_Engine::CheckInstance())
//		return ERR_CLASS_INSTANCE;
//
//	PxVec3 center;// (PxVec3)pose;// PxVec3(-20.31, -422.0, -766.59); // x, y, z reel 
//	center.x = pose.Tx;
//	center.y = pose.Ty;
//	center.z = pose.Tz;
//	PxVec3 rotation;// (PxVec3)pose;// PxVec3(-20.31, -422.0, -766.59); // x, y, z reel 
//	rotation.x = rot.Rx;
//	rotation.y = rot.Ry;
//	rotation.z = rot.Rz;
//	PxVec3 dim_carton;// = dimensions;// PxVec3(104.5, 145.0, 100.0); // x, y, z reel
//	dim_carton.x = dimensions.width;
//	dim_carton.y = dimensions.length;
//	dim_carton.z = dimensions.height;
//
//	std::string MeshPath((char*)LStrBuf(*path), 0, LStrLen(*path));
//
//	PhysX_Engine::GetInstance().create_box(dim_carton, center, rotation, MeshPath);
//	uint32_t Nb_active = 1;
//	PhysX_Engine::GetInstance().stepPhysics(false, Nb_active);
//}
//
//__declspec(dllexport) int __cdecl CreateBox(const Dims boxInsideSizeM_arg, const Position boxThickSideBottomM_arg, const Position boxInsidePoseMpos_arg, const Orientation boxInsidePoseMrot_arg, const float bigger_wall_arg) {
//	if (!PhysX_Engine::CheckInstance()) {
//		return ERR_CLASS_INSTANCE;
//	}
//
//	//PxVec3 boxInsidePoseMpos;// (PxVec3)pose;// PxVec3(-20.31, -422.0, -766.59); // x, y, z reel 
//	//boxInsidePoseMpos.x = boxInsidePoseMpos_arg.Tx;
//	//boxInsidePoseMpos.y = boxInsidePoseMpos_arg.Ty;
//	//boxInsidePoseMpos.z = boxInsidePoseMpos_arg.Tz;
//	//PxVec3 boxInsidePoseMrot;// (PxVec3)pose;// PxVec3(-20.31, -422.0, -766.59); // x, y, z reel 
//	//boxInsidePoseMrot.x = boxInsidePoseMrot_arg.Rx;
//	//boxInsidePoseMrot.y = boxInsidePoseMrot_arg.Ry;
//	//boxInsidePoseMrot.z = boxInsidePoseMrot_arg.Rz;
//	//PxVec3 boxInsideSizeM;// = dimensions;// PxVec3(104.5, 145.0, 100.0); // x, y, z reel
//	//boxInsideSizeM.x = boxInsideSizeM_arg.width;
//	//boxInsideSizeM.y = boxInsideSizeM_arg.length;
//	//boxInsideSizeM.z = boxInsideSizeM_arg.height;
//	//PxVec2 boxThickSideBottomM;
//	//boxThickSideBottomM.x = 1;
//	//boxThickSideBottomM.y = 1;
//	////boxThickSideBottomM.x = boxThickSideBottomM_arg.thickness_top;
//	////boxThickSideBottomM.y = boxThickSideBottomM_arg.thickness_bot;
//	//float bigger_wall = bigger_wall_arg;
//
//	PxVec3 boxInsideSizeM = PxVec3(300, 300, 300);
//	PxVec3 boxInsidePoseMpos = PxVec3(0, 0, 0);
//	//PxVec3 boxInsidePoseMpos = PxVec3(46.4935, -823.521, -500);
//	//PxVec3 boxInsidePoseMrot = PxVec3(337.809, 2.1306, -59.0498);
//	//PxVec3 boxInsidePoseMrot = PxVec3(0,0,0);
//	PxVec3 boxInsidePoseMrot = PxVec3(-20, 0, 0);
//	PxVec2 boxThickSideBottomM = PxVec2(1.f, 1.f);
//	float bigger_wall = 0.f;
//	std::cout << "INPUT : " << std::endl;
//	std::cout << "boxInsidePoseMpos : " << boxInsidePoseMpos.x << " - " << boxInsidePoseMpos.y << " - " << boxInsidePoseMpos.z << std::endl;
//	std::cout << "boxInsidePoseMrot : " << boxInsidePoseMrot.x << " - " << boxInsidePoseMrot.y << " - " << boxInsidePoseMrot.z << std::endl;
//	std::cout << "boxInsideSizeM : " << boxInsideSizeM.x << " - " << boxInsideSizeM.y << " - " << boxInsideSizeM.z << std::endl;
//	std::cout << "boxThickSideBottomM : " << boxThickSideBottomM_arg.Tx << " - " << boxThickSideBottomM_arg.Ty << std::endl;
//	std::cout << "bigger_wall : " << bigger_wall_arg << std::endl;
//
//	PhysX_Engine::GetInstance().create_box(boxInsideSizeM, boxThickSideBottomM, boxInsidePoseMpos, boxInsidePoseMrot, bigger_wall);
//	uint32_t Nb_active = 1;
//	PhysX_Engine::GetInstance().stepPhysics(false, Nb_active);
//	return 0;
//
//}
//
//__declspec(dllexport) int __cdecl Init_object(LStrHandle path, int size_obj) {
//	if (!PhysX_Engine::CheckInstance())
//		return ERR_CLASS_INSTANCE;
//	std::string path_arg((char*)LStrBuf(*path), 0, LStrLen(*path));
//	
//	PhysX_Engine::GetInstance().init_object(path_arg, size_obj,  PhysX_Engine::GetInstance().gPhysics, PhysX_Engine::GetInstance().gMaterial, PhysX_Engine::GetInstance().gCooking);
//
//	return 0;
//}
//
//
//__declspec(dllexport) int __cdecl DeleteAllActorsOut() {
//	if (!PhysX_Engine::CheckInstance())
//		return ERR_CLASS_INSTANCE;
//	PhysX_Engine::GetInstance().DeleteAllActorsOut();
//}
//
//
//__declspec(dllexport) int __cdecl DeleteActor(const int ID_box, const int ID_Type, const int index) {
//	if (!PhysX_Engine::CheckInstance())
//		return ERR_CLASS_INSTANCE;
//	PhysX_Engine::GetInstance().DeleteActor(ID_box, ID_Type, index);
//}
//
//
//
//__declspec(dllexport) int __cdecl FillBox(const int ID_box, const int ID_Type, const int Nb_object) {
//	if (!PhysX_Engine::CheckInstance())
//		return ERR_CLASS_INSTANCE;
//	PhysX_Engine::GetInstance().fill_box(ID_box, ID_Type, Nb_object);
//	uint32_t Nb_active = 1;
//	PhysX_Engine::GetInstance().stepPhysics(false, Nb_active);
//}
//
//__declspec(dllexport) int __cdecl FillBox2(const int ID_box, const int ID_Type, const int Nb_object) {
//	if (!PhysX_Engine::CheckInstance())
//		return ERR_CLASS_INSTANCE;
//	PhysX_Engine::GetInstance().fill_box_2(ID_box, ID_Type, Nb_object);
//	uint32_t Nb_active = 1;
//	PhysX_Engine::GetInstance().stepPhysics(false, Nb_active);
//}
//
//
//__declspec(dllexport) int __cdecl DoAllStep() {
//	if (!PhysX_Engine::CheckInstance())
//		return ERR_CLASS_INSTANCE;
//	PhysX_Engine::GetInstance().DoAllStep();
//	return 0;
//}
//
//__declspec(dllexport) unsigned int __cdecl DoStep() {
//	if (!PhysX_Engine::CheckInstance())
//		return ERR_CLASS_INSTANCE;
//	uint32_t Nb_active = 1; //not 0
//
//	PhysX_Engine::GetInstance().stepPhysics(false, Nb_active);
//
//	return Nb_active;
//
//}
//
//__declspec(dllexport) unsigned int __cdecl DoStep_N(uint32_t Nb_step) {
//	if (!PhysX_Engine::CheckInstance())
//		return ERR_CLASS_INSTANCE;
//	uint32_t Nb_active = 1; //not 0
//
//	PhysX_Engine::GetInstance().stepPhysics(false, Nb_active, Nb_step);
//
//	return Nb_active;
//}
//
//__declspec(dllexport) int __cdecl GetAllItemsPosition(int ID_box, int ID_Type, arr1Df_LV Tx, arr1Df_LV Ty, arr1Df_LV Tz, arr1Df_LV q0, arr1Df_LV q1, arr1Df_LV q2, arr1Df_LV q3, arr1Df_LV is_valid){
//	if (!PhysX_Engine::CheckInstance())
//		return ERR_CLASS_INSTANCE;
//	PhysX_Engine::GetInstance().DeleteAllActorsOut();
//	PhysX_Engine::GetInstance().GetObjetsPositionActors(ID_box, ID_Type, Tx, Ty, Tz, q0, q1, q2, q3, is_valid);
//
//	return 0;
//}
//
//__declspec(dllexport) int __cdecl Shuffle(int ID_box, float force_value) {
//	if (!PhysX_Engine::CheckInstance())
//		return ERR_CLASS_INSTANCE;
//	PhysX_Engine::GetInstance().Shuffle(ID_box, force_value);
//
//	return 0;
//}