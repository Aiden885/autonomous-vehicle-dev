#ifndef	_VehIntentionAndRequest_H_
#define	_VehIntentionAndRequest_H_

#include <stdint.h>
#include <string>
#include <memory>
#include <vector>
#include "common.h"
#include "rsi.h"

namespace Ivics
{
typedef enum ReqStatus {
	ReqStatus_unknown = 0,
	ReqStatus_request = 1,
	ReqStatus_comfirmed = 2,
	ReqStatus_cancel = 3,
	ReqStatus_complete = 4
} EReqStatus;

typedef enum ReqInfo_PR {
	ReqInfo_PR_NOTHING,	/* No components present */
	ReqInfo_PR_laneChange,
	ReqInfo_PR_clearTheWay,
	ReqInfo_PR_signalPriority,
	ReqInfo_PR_sensorSharing,
	ReqInfo_PR_parking
	/* Extensions may appear below */
} EReqInfo_PR;

typedef uint8_t LaneID_t;

typedef struct Req_LaneChange {
	SNodeReferenceID upstreamNode;
	SNodeReferenceID downstreamNode;
	LaneID_t	 targetLane;
} SReqLaneChange;

typedef struct Req_ClearTheWay {
	SNodeReferenceID upstreamNode;
	SNodeReferenceID downstreamNode;
	LaneID_t	 targetLane;
	std::shared_ptr<struct ReferencePath> relatedPath	/* OPTIONAL */;
	std::shared_ptr<struct DDateTime> tBegin	/* OPTIONAL */;
	std::shared_ptr<struct DDateTime> tEnd	/* OPTIONAL */;
} SReqClearTheWay;

typedef enum Maneuver {
	Maneuver_maneuverStraight = 0,
	Maneuver_maneuverLeftTurn = 1,
	Maneuver_maneuverRightTurn = 2,
	Maneuver_maneuverUTurn = 3
} EManeuver;

typedef struct MovementEx {
	SNodeReferenceID remoteIntersection;
	std::shared_ptr<long> phaseId	/* OPTIONAL */;
	std::shared_ptr<EManeuver> turn_direction	/* OPTIONAL */;
} SMovementEx;

typedef struct Req_SensorSharing {
	SReferencePathList detectArea;
} SReqSensorSharing;

typedef struct Req_SignalPriority {
	SNodeReferenceID intersectionId;
	SMovementEx requiredMov;
	std::shared_ptr<TimeOffset_t> estimatedArrivalTime	/* OPTIONAL */;
	std::shared_ptr<long> distance2Intersection	/* OPTIONAL */;
} SReqSignalPriority;

typedef struct Req_ParkingArea {
	SVehicleClassification vehicleClass;
	uint16_t req;
	std::shared_ptr<uint16_t> parkingType	/* OPTIONAL */;
	std::shared_ptr<long> expectedParkingSlotID	/* OPTIONAL */;
} SReqParkingArea;

typedef struct ReqInfo {
	EReqInfo_PR present;
	union UReqInfo {
		SReqLaneChange laneChange;
		SReqClearTheWay clearTheWay;
		SReqSignalPriority signalPriority;
		SReqSensorSharing sensorSharing;
		SReqParkingArea parking;
	} choice;
} SReqInfo;

/**
 * @brief 定义车辆发出的请求信息。
 * 包括此次请求消息的标识ID、请求消息的操作状态，可选字段包括请求的优先级、目标车辆的临时标识、目标RSU的临时标识、请求的内容、消息的有效期等。
 *
 * @details
 */
typedef struct DriveRequest {
	long reqID;                             // local ID of this request. same request in serial VIR messages should keep the same reqID
	EReqStatus status;
	std::shared_ptr<uint8_t> reqPriority;   // OPTIONAL, reqPriority OCTET STRING (SIZE(1)), One-bit string. The lower five bits are reserved and shall be set to zero
	std::shared_ptr<std::string> targetVeh	/* OPTIONAL */;
	std::shared_ptr<std::string> targetRSU	/* OPTIONAL */;
	std::shared_ptr<struct ReqInfo> info	/* OPTIONAL */;
	std::shared_ptr<TimeOffset_t> lifeTime	/* OPTIONAL */;
} SDriveRequest;


/**
 * @brief 定义车辆的行驶计划和请求信息。包括车辆当前在地图中的位置、规划的行驶路线、相关的驾驶行为、请求消息等
 */
typedef struct IARData {
	using DrvReqList = std::vector<struct DriveRequest>;
	std::shared_ptr<SPathPlanningPoint>	currentPos;	    // OPTIONAL, current position in MAP
	std::shared_ptr<struct PathPlanning> path_Planning;	// OPTIONAL，real time path planning that is shared with neighbors,list in chronological order
	std::shared_ptr<EDriveBehavior>	currentBehavior;    // OPTIONAL，drive behavior related to the path planning;
	std::shared_ptr<DrvReqList> reqs;                   // OPTIONAL, 
} SIARData;

/**
 * @brief 车辆意图及请求消息。用来进行车辆驾驶意图、优先请求、协作请求等信息的传递。
 *
 */
typedef struct VIR {
	u_int8_t msgCnt;
	std::string id;       // temperary vehicle ID, same as id in BSM
	uint64_t secMark;
	SPosition3D refPos;   // vehicle real position relates to secMark
	SIARData intAndReq;   // vehicle intention and request

} VIR;



}//namespace ivics




#endif	/* _BasicSafetyMessage_H_ */