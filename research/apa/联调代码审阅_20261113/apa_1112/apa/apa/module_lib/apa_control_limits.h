#pragma once

#include <string>
#include "apa_config.pb.h"
extern "C" {
#include "apaMocoParkingControl.h"
}

namespace pangu::modules {

bool BuildApaControlCalibration(const ApaParams& params,
    ApaMocoParkingControlParam_t* control, std::string* error);

bool ApplyApaActuationLimits(const ApaParams& params,
    ApaMocoParkingControlParam_t* control, std::string* error);
ApaActuationLimits DescribeApaActuationLimits(const ApaMocoParkingControlParam_t& control);

}
