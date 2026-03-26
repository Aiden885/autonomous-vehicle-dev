#ifndef _UTILS_
#define _UTILS_

#include "dataType.h"

class DETECTION_ROW
{
  public:
    DETECTBOX xywh;   // np.float
    float confidence; // float
    int class_num;
    DETECTBOX to_xyah() const;
    DETECTBOX to_tlbr() const;
};

typedef std::vector<DETECTION_ROW> DETECTIONS;

#endif