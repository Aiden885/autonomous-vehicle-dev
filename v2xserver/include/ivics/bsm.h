#ifndef	_BasicSafetyMessage_H_
#define	_BasicSafetyMessage_H_

#include <stdint.h>
#include <string>
#include <memory>
#include <vector>
#include "common.h"
namespace Ivics
{
//
typedef struct PositionalAccuracy {
	u_int8_t semiMajor; 
	u_int8_t semiMinor; 
	u_int64_t orientation; 
} SPositionalAccuracy;

//BrakeSystemStatus
typedef enum BrakePedalStatus {
	BrakePedalStatus_unavailable	= 0,
	BrakePedalStatus_off	= 1,
	BrakePedalStatus_on	= 2
} EBrakePedalStatus;

typedef enum BrakeAppliedStatus {
	BrakeAppliedStatus_unavailable	= 0,
	BrakeAppliedStatus_leftFront	= 1,
	BrakeAppliedStatus_leftRear	= 2,
	BrakeAppliedStatus_rightFront	= 3,
	BrakeAppliedStatus_rightRear	= 4
} EBrakeAppliedStatus;

typedef enum TractionControlStatus {
	TractionControlStatus_unavailable	= 0,
	TractionControlStatus_off	= 1,
	TractionControlStatus_on	= 2,
	TractionControlStatus_engaged	= 3
} ETractionControlStatus;

typedef enum AntiLockBrakeStatus {
	AntiLockBrakeStatus_unavailable	= 0,
	AntiLockBrakeStatus_off	= 1,
	AntiLockBrakeStatus_on	= 2,
	AntiLockBrakeStatus_engaged	= 3
} EAntiLockBrakeStatus;

typedef enum StabilityControlStatus {
	StabilityControlStatus_unavailable	= 0,
	StabilityControlStatus_off	= 1,
	StabilityControlStatus_on	= 2,
	StabilityControlStatus_engaged	= 3
} EStabilityControlStatus;

typedef enum BrakeBoostApplied {
	BrakeBoostApplied_unavailable	= 0,
	BrakeBoostApplied_off	= 1,
	BrakeBoostApplied_on	= 2
} EBrakeBoostApplied;

typedef enum AuxiliaryBrakeStatus {
	AuxiliaryBrakeStatus_unavailable	= 0,
	AuxiliaryBrakeStatus_off	= 1,
	AuxiliaryBrakeStatus_on	= 2,
	AuxiliaryBrakeStatus_reserved	= 3
} EAuxiliaryBrakeStatus;

typedef struct BrakeSystemStatus {
	std::shared_ptr<EBrakePedalStatus> brakePadel	/* OPTIONAL */;
	std::shared_ptr<EBrakeAppliedStatus> wheelBrakes	/* OPTIONAL */;
	std::shared_ptr<ETractionControlStatus> traction	/* OPTIONAL */;
	std::shared_ptr<EAntiLockBrakeStatus> abs	/* OPTIONAL */;
	std::shared_ptr<EStabilityControlStatus> scs	/* OPTIONAL */;
	std::shared_ptr<EBrakeBoostApplied> brakeBoost	/* OPTIONAL */;
	std::shared_ptr<EAuxiliaryBrakeStatus> auxBrakes	/* OPTIONAL */;
} SBrakeSystemStatus;

//VehicleSafetyExtensions
typedef enum VehicleEventFlags {
	VehicleEventFlags_eventHazardLights	= 0,
	VehicleEventFlags_eventStopLineViolation	= 1,
	VehicleEventFlags_eventABSactivated	= 2,
	VehicleEventFlags_eventTractionControlLoss	= 3,
	VehicleEventFlags_eventStabilityControlactivated	= 4,
	VehicleEventFlags_eventHazardousMaterials	= 5,
	VehicleEventFlags_eventReserved1	= 6,
	VehicleEventFlags_eventHardBraking	= 7,
	VehicleEventFlags_eventLightsChanged	= 8,
	VehicleEventFlags_eventWipersChanged	= 9,
	VehicleEventFlags_eventFlatTire	= 10,
	VehicleEventFlags_eventDisabledVehicle	= 11,
	VehicleEventFlags_eventAirBagDeployment	= 12
} EVehicleEventFlags;

typedef struct PathPrediction {
	uint64_t radiusOfCurve;//Units of 10cm 
	double	confidence;//percent
} SPathPrediction;

typedef struct VehicleSafetyExtensions {
	std::shared_ptr<EVehicleEventFlags> events;
    std::shared_ptr<SPathHistory> pathHistory;
	std::shared_ptr<SPathPrediction> pathPrediction;
	std::shared_ptr<EExteriorLights> lights;
} SVehicleSafetyExtensions;

typedef enum ResponseType {
	ResponseType_notInUseOrNotEquipped	= 0,
	ResponseType_emergency	= 1,
	ResponseType_nonEmergency	= 2,
	ResponseType_pursuit	= 3,
	ResponseType_stationary	= 4,
	ResponseType_slowMoving	= 5,
	ResponseType_stopAndGoMovement	= 6
} EResponseType;

typedef enum SirenInUse {
	SirenInUse_unavailable	= 0,
	SirenInUse_notInUse	= 1,
	SirenInUse_inUse	= 2,
	SirenInUse_reserved	= 3
} ESirenInUse;

typedef enum LightbarInUse {
	LightbarInUse_unavailable	= 0,
	LightbarInUse_notInUse	= 1,
	LightbarInUse_inUse	= 2,
	LightbarInUse_yellowCautionLights	= 3,
	LightbarInUse_schooldBusLights	= 4,
	LightbarInUse_arrowSignsActive	= 5,
	LightbarInUse_slowMovingVehicle	= 6,
	LightbarInUse_freqStops	= 7
} ELightbarInUse;

typedef struct VehicleEmergencyExtensions {
    std::shared_ptr<EResponseType> responseType;
    std::shared_ptr<ESirenInUse> sirenUse;
    std::shared_ptr<ELightbarInUse> lightsUse;
} SVehicleEmergencyExtensions;

// 基本安全消息（Basic Safety Message）
typedef struct BasicSafetyMessage
{
    u_int8_t msgCnt;
    std::string id;                  // temperary vehicle ID
    u_int64_t secMark;               // 时间戳，精确到毫秒
    ETimeConfidence timeConfidence;  
    struct Position3D pos;           // 位置 
    std::shared_ptr<SPositionalAccuracy> posAccuracy;
    std::shared_ptr<SPositionConfidenceSet> posConfidence;
    std::shared_ptr<ETransmissionState> transmission;
    double speed;                    // 速度，代码:m/s
    double heading;                  // 车头朝向，unit:radian , take east as 0 , anticlockwise
    std::shared_ptr<double> angle;	/* OPTIONAL -189 to +189 degrees */
    std::shared_ptr<SMotionConfidenceSet> motionCfd;	/* OPTIONAL */
    struct AccelerationSet4Way accelSet;  //加速度
    struct BrakeSystemStatus brakes;
    struct VehicleSize	size;
    struct VehicleClassification vehicleClass;
    std::shared_ptr<SVehicleSafetyExtensions> safetyExt;	/* OPTIONAL */
    std::shared_ptr<SVehicleEmergencyExtensions> emergencyExt;	/* OPTIONAL */
} BSM;




}//namespace ivics




#endif	/* _BasicSafetyMessage_H_ */