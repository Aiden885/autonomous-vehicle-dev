#ifndef	_RoadsideSafetyMessage_H_
#define	_RoadsideSafetyMessage_H_

#include <stdint.h>
#include <string>
#include <memory>
#include <vector>
#include "common.h"
namespace Ivics
{
//
typedef enum EventSource {
	EventSource_unknown	= 0,
	EventSource_police	= 1,
	EventSource_government	= 2,
	EventSource_meteorological	= 3,
	EventSource_internet	= 4,
	EventSource_detection	= 5
} EEventSource;

//
typedef struct RSITimeDetails {
	std::shared_ptr<uint64_t> startTime;
	std::shared_ptr<uint64_t> endTime	/* OPTIONAL */;
	std::shared_ptr<uint64_t> endTimeConfidence	/* OPTIONAL */;
} SRSITimeDetails;

//RTEData
typedef struct RTEData {
	long rteId;
	int eventType;
	EEventSource eventSource;
	std::shared_ptr<SPosition3D> eventPos	/* OPTIONAL */;
	std::shared_ptr<uint64_t> eventRadius	/* OPTIONAL */; 
	std::shared_ptr<std::string> description	/* OPTIONAL */;
	std::shared_ptr<RSITimeDetails> timeDetails	/* OPTIONAL */;
	std::shared_ptr<uint8_t> priority	/* OPTIONAL */;
	std::shared_ptr<ReferencePathList> referencePaths	/* OPTIONAL */;
	std::shared_ptr<ReferenceLinkList> referenceLinks	/* OPTIONAL */;
	std::shared_ptr<uint8_t> eventConfidence	/* OPTIONAL */;	
} SRTEData;

typedef struct RTEList {
	std::vector<struct RTEData> list;
} SRTEList;

//RTS
typedef enum AuxiliarySignVehicleType {
	AuxiliarySignVehicleType_restrictedFromBus	= 0,
	AuxiliarySignVehicleType_vehicle	= 1,
	AuxiliarySignVehicleType_truck	= 2,
	AuxiliarySignVehicleType_tractor	= 3,
	AuxiliarySignVehicleType_private	= 4
} EAuxiliarySignVehicleType;

typedef enum AuxiliarySignDirection {
	AuxiliarySignDirection_straight	= 0,
	AuxiliarySignDirection_leftAndRight	= 1,
	AuxiliarySignDirection_right	= 2,
	AuxiliarySignDirection_left	= 3,
	AuxiliarySignDirection_leftFrontTurn	= 4,
	AuxiliarySignDirection_rightFronTurn	= 5,
	AuxiliarySignDirection_rightRearTurn	= 6,
	AuxiliarySignDirection_leftRearTurn	= 7
} EAuxiliarySignDirection;

typedef struct AuxiliarySign {
	std::shared_ptr<EAuxiliarySignVehicleType> signWithVehicleType	/* OPTIONAL */;
	std::shared_ptr<EAuxiliarySignDirection> signDirection	/* OPTIONAL */;
} SAuxiliarySign;

typedef enum SignType {
	SignType_UNKNOWN	= 0,
	SignType_TRAFFICACCIDENT	= 1,
} ESignType;

typedef struct RTSData {
	long rtsId;
	ESignType signType;
	std::shared_ptr<SPosition3D> signPos	/* OPTIONAL */;
	std::shared_ptr<std::string> description	/* OPTIONAL */;
	std::shared_ptr<RSITimeDetails> timeDetails	/* OPTIONAL */;
	std::shared_ptr<uint8_t> priority	/* OPTIONAL */;
	std::shared_ptr<ReferencePathList> referencePaths	/* OPTIONAL */;
	std::shared_ptr<ReferenceLinkList> referenceLinks	/* OPTIONAL */;
	std::shared_ptr<SAuxiliarySign> auxiliarySign	/* OPTIONAL */;
}SRTSData;

typedef struct RTSList {
	std::vector<struct RTSData> list;
} SRTSList;

typedef struct RoadSideInformation {
	u_int8_t msgCnt;                          //0-127
	std::shared_ptr<u_int64_t> moy;          //minute of the year
	std::string id;
	struct Position3D refPos;
	std::shared_ptr<SRTEList> rtes;         // 道路交通事件信息
	std::shared_ptr<SRTSList> rtss;         // 道路交通标志信息
} RSI;



}//namespace ivics




#endif	/* _BasicSafetyMessage_H_ */