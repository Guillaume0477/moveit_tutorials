#pragma once

#define NO_ERROR 0
#define ERR_CLASS_INSTANCE -1
#define ERR_BAD_INPUT -2
#define ERR_EXCEPTION_CATCHED -3

#include <string>

inline std::string GetErrorFromCode(const int errorCode) {
	switch (errorCode) {
	case 0:
		return "No error";
	case -1:
		return "PhysX Error - Instance not created";
	case -2:
		return "PhysX Error - Bad input";
	case -3:
		return "PhysX Error - Exception catched";


	default:
		return "PhysX Error - Error code unknown";
	}
}