#ifndef APA_VEHICLE_DEFAULTS_H
#define APA_VEHICLE_DEFAULTS_H

/* Shared calibration: conf/apa_vehicle_profile.json. Generated C constants and
 * Bridge data use the same checked content hash. */
#include "apaVehicleProfileGenerated.h"
#include <math.h>
/* Planning radius reserves tracking room; control retains full steering lock. */
#define APA_VEHICLE_LENGTH_M (APA_VEHICLE_REAR_TO_FRONT_M + APA_VEHICLE_REAR_TO_BACK_M)
#define APA_VEHICLE_FRONT_OVERHANG_M (APA_VEHICLE_REAR_TO_FRONT_M - APA_VEHICLE_WHEELBASE_M)

#endif
