/* Auto-generated header */
#ifndef NEWACCPRO2_H
#define NEWACCPRO2_H


// INCLUDES_START

#include "base_types.hpp"
 #include <type_traits>
#include <utility>
#include "GlobalContextTypes.hpp"
#include "base_types.hpp"
#include <algorithm>

// INCLUDES_END


// TYPEDEFS_START

// TYPEDEFS_END


// MACROS_START

// MACROS_END


// ENUMS_START

enum class DecisionResult {
    none = 0,
    r1 = 1,
    r2 = 2,
    r3 = 3,
    r4 = 4,
    r5 = 5,
    r6 = 6,
    r7 = 7,
    r8 = 8
};
 enum class DriverCommand {
    none = 0,
    inc = 1,
    dec = 2,
    cancel = 3,
    brake = 4,
    exit = 5
};
 enum class SpeedSubstate {
    suitable = 0,
    low = 1,
    high = 2
};
 enum class SystemState {
    s0 = 0,
    s1 = 1,
    s2 = 2,
    s3 = 3,
    s4 = 4,
    s5 = 5,
    s6 = 6
};
 enum class SystemSubstate {
    s0 = 0,
    s1 = 1,
    s2 = 2,
    s3 = 3,
    s4 = 4,
    s5 = 5,
    s6 = 6,
    s7 = 7
};

// ENUMS_END


// STRUCTS_START

// STRUCTS_END


// FUNCTIONS_START

// FUNCTIONS_END


#endif