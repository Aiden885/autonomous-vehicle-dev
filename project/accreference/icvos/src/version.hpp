// Author:jary
//
#pragma once
#include <string>

#define VERSION_MAIN 2
#define VERSION_MAJOR 5
#define VERSION_MINOR 0

#define STR_EXP(x) #x
#define STR(x) STR_EXP(x)

#define VERSION_STRING STR(VERSION_MAIN) "." STR(VERSION_MAJOR) "." STR(VERSION_MINOR)
