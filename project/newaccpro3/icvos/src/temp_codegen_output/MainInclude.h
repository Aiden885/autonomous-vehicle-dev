/* Auto-generated header */
#ifndef MAININCLUDE_H
#define MAININCLUDE_H


// INCLUDES_START

#include "base_types.hpp"
#include "algorithm"
#include "cmath"
#include "math.h"
#include "type_traits"
#include "utility"

// INCLUDES_END


// TYPEDEFS_START

// TYPEDEFS_END


// MACROS_START

// MACROS_END


// ENUMS_START

typedef enum __DecisionResult {
    none = 0,
    r1 = 1,
    r2 = 2,
    r3 = 3,
    r4 = 4,
    r5 = 5,
    r6 = 6,
    r7 = 7,
    r8 = 8
} DecisionResult;
typedef enum __DriverCommand {
    none = 0,
    inc = 1,
    dec = 2,
    cancel = 3,
    brake = 4,
    exit = 5
} DriverCommand;
typedef enum __SpeedSubstate {
    suitable = 0,
    low = 1,
    high = 2
} SpeedSubstate;
typedef enum __SystemState {
    s0 = 0,
    s1 = 1,
    s2 = 2,
    s3 = 3,
    s4 = 4,
    s5 = 5,
    s6 = 6
} SystemState;
typedef enum __SystemSubstate {
    s0 = 0,
    s1 = 1,
    s2 = 2,
    s3 = 3,
    s4 = 4,
    s5 = 5,
    s6 = 6,
    s7 = 7
} SystemSubstate;

// ENUMS_END


// STRUCTS_START

// STRUCTS_END


// FUNCTIONS_START

// FUNCTIONS_END


#endif