#ifndef	_Common_H_
#define	_Common_H_

#include <stdint.h>
#include <string>
#include <memory>
#include <vector>

namespace Ivics
{
	typedef double Confidence_t;
	typedef uint64_t TimeOffset_t;
	
	typedef enum TimeConfidence {
	TimeConfidence_unavailable	= 0,
	TimeConfidence_time_100_000	= 1,
	TimeConfidence_time_050_000	= 2,
	TimeConfidence_time_020_000	= 3,
	TimeConfidence_time_010_000	= 4,
	TimeConfidence_time_002_000	= 5,
	TimeConfidence_time_001_000	= 6,
	TimeConfidence_time_000_500	= 7,
	TimeConfidence_time_000_200	= 8,
	TimeConfidence_time_000_100	= 9,
	TimeConfidence_time_000_050	= 10,
	TimeConfidence_time_000_020	= 11,
	TimeConfidence_time_000_010	= 12,
	TimeConfidence_time_000_005	= 13,
	TimeConfidence_time_000_002	= 14,
	TimeConfidence_time_000_001	= 15,
	TimeConfidence_time_000_000_5	= 16,
	TimeConfidence_time_000_000_2	= 17,
	TimeConfidence_time_000_000_1	= 18,
	TimeConfidence_time_000_000_05	= 19,
	TimeConfidence_time_000_000_02	= 20,
	TimeConfidence_time_000_000_01	= 21,
	TimeConfidence_time_000_000_005	= 22,
	TimeConfidence_time_000_000_002	= 23,
	TimeConfidence_time_000_000_001	= 24,
	TimeConfidence_time_000_000_000_5	= 25,
	TimeConfidence_time_000_000_000_2	= 26,
	TimeConfidence_time_000_000_000_1	= 27,
	TimeConfidence_time_000_000_000_05	= 28,
	TimeConfidence_time_000_000_000_02	= 29,
	TimeConfidence_time_000_000_000_01	= 30,
	TimeConfidence_time_000_000_000_005	= 31,
	TimeConfidence_time_000_000_000_002	= 32,
	TimeConfidence_time_000_000_000_001	= 33,
	TimeConfidence_time_000_000_000_000_5	= 34,
	TimeConfidence_time_000_000_000_000_2	= 35,
	TimeConfidence_time_000_000_000_000_1	= 36,
	TimeConfidence_time_000_000_000_000_05	= 37,
	TimeConfidence_time_000_000_000_000_02	= 38,
	TimeConfidence_time_000_000_000_000_01	= 39
} ETimeConfidence;

typedef struct Position3D {
	double x; //unit:m
	double y; //unit:m
	std::shared_ptr<double> z;/* OPTIONAL */
	
} SPosition3D;

// typedef struct PositionOffsetLLV {
// 	PositionOffsetLL_t	 offsetLL;
// 	struct VerticalOffset	*offsetV	/* OPTIONAL */;
// } PositionOffsetLLV_t;

typedef enum PositionConfidence {
	PositionConfidence_unavailable	= 0,
	PositionConfidence_a500m	= 1,
	PositionConfidence_a200m	= 2,
	PositionConfidence_a100m	= 3,
	PositionConfidence_a50m	= 4,
	PositionConfidence_a20m	= 5,
	PositionConfidence_a10m	= 6,
	PositionConfidence_a5m	= 7,
	PositionConfidence_a2m	= 8,
	PositionConfidence_a1m	= 9,
	PositionConfidence_a50cm	= 10,
	PositionConfidence_a20cm	= 11,
	PositionConfidence_a10cm	= 12,
	PositionConfidence_a5cm	= 13,
	PositionConfidence_a2cm	= 14,
	PositionConfidence_a1cm	= 15
} EPositionConfidence;

typedef enum ElevationConfidence {
	ElevationConfidence_unavailable	= 0,
	ElevationConfidence_elev_500_00	= 1,
	ElevationConfidence_elev_200_00	= 2,
	ElevationConfidence_elev_100_00	= 3,
	ElevationConfidence_elev_050_00	= 4,
	ElevationConfidence_elev_020_00	= 5,
	ElevationConfidence_elev_010_00	= 6,
	ElevationConfidence_elev_005_00	= 7,
	ElevationConfidence_elev_002_00	= 8,
	ElevationConfidence_elev_001_00	= 9,
	ElevationConfidence_elev_000_50	= 10,
	ElevationConfidence_elev_000_20	= 11,
	ElevationConfidence_elev_000_10	= 12,
	ElevationConfidence_elev_000_05	= 13,
	ElevationConfidence_elev_000_02	= 14,
	ElevationConfidence_elev_000_01	= 15
} EElevationConfidence;

typedef struct PositionConfidenceSet {
	EPositionConfidence pos;
	std::shared_ptr<EElevationConfidence> elevation	/* OPTIONAL */;
	
} SPositionConfidenceSet;


typedef enum TransmissionState {
	TransmissionState_neutral	= 0,
	TransmissionState_park	= 1,
	TransmissionState_forwardGears	= 2,
	TransmissionState_reverseGears	= 3,
	TransmissionState_reserved1	= 4,
	TransmissionState_reserved2	= 5,
	TransmissionState_reserved3	= 6,
	TransmissionState_unavailable	= 7
} ETransmissionState;

//MotionConfidenceSet
typedef enum SpeedConfidence {
	SpeedConfidence_unavailable	= 0,
	SpeedConfidence_prec100ms	= 1,
	SpeedConfidence_prec10ms	= 2,
	SpeedConfidence_prec5ms	= 3,
	SpeedConfidence_prec1ms	= 4,
	SpeedConfidence_prec0_1ms	= 5,
	SpeedConfidence_prec0_05ms	= 6,
	SpeedConfidence_prec0_01ms	= 7
} ESpeedConfidence;

typedef enum HeadingConfidence {
	HeadingConfidence_unavailable	= 0,
	HeadingConfidence_prec10deg	= 1,
	HeadingConfidence_prec05deg	= 2,
	HeadingConfidence_prec01deg	= 3,
	HeadingConfidence_prec0_1deg	= 4,
	HeadingConfidence_prec0_05deg	= 5,
	HeadingConfidence_prec0_01deg	= 6,
	HeadingConfidence_prec0_0125deg	= 7
} EHeadingConfidence;

typedef enum SteeringWheelAngleConfidence {
	SteeringWheelAngleConfidence_unavailable	= 0,
	SteeringWheelAngleConfidence_prec2deg	= 1,
	SteeringWheelAngleConfidence_prec1deg	= 2,
	SteeringWheelAngleConfidence_prec0_02deg	= 3
} ESteeringWheelAngleConfidence;

typedef struct MotionConfidenceSet {
	std::shared_ptr<ESpeedConfidence> speedCfd;	/* OPTIONAL */
	std::shared_ptr<EHeadingConfidence>	headingCfd	/* OPTIONAL */;
	std::shared_ptr<ESteeringWheelAngleConfidence> steerCfd	/* OPTIONAL */;
} SMotionConfidenceSet;

//AccelerationSet4Way
typedef struct AccelerationSet4Way {
	double Long;
	double lat;
	double vert;
	double yaw;
} SAccelerationSet4Way;

//VehicleSize
typedef struct VehicleSize {
	double width;
	double length;
	std::shared_ptr<double> height	/* OPTIONAL */;
} SVehicleSize;

//VehicleClassification
typedef struct VehicleClassification {
	uint8_t	classification;
    std::shared_ptr<uint8_t> fuelType;
} SVehicleClassification;

typedef struct DDateTime {

	std::shared_ptr<uint16_t> year	/* OPTIONAL */;
	std::shared_ptr<uint8_t> month	/* OPTIONAL */;
	std::shared_ptr<uint8_t> day	/* OPTIONAL */;
	std::shared_ptr<uint8_t> hour	/* OPTIONAL */;
	std::shared_ptr<uint8_t> minute	/* OPTIONAL */;
	std::shared_ptr<uint64_t> second	/* OPTIONAL */;
	std::shared_ptr<uint16_t> offset	/* OPTIONAL */;
} SDDateTime;

typedef struct FullPositionVector {
	std::shared_ptr<SDDateTime> utcTime	/* OPTIONAL */;
	SPosition3D pos;
    std::shared_ptr<double> heading;/* OPTIONAL */
    std::shared_ptr<ETransmissionState> transmission;/* OPTIONAL */
	std::shared_ptr<double> speed	/* OPTIONAL */;
    std::shared_ptr<SPositionConfidenceSet> posAccuracy;/* OPTIONAL */
	std::shared_ptr<ETimeConfidence> timeConfidence	/* OPTIONAL */;
	std::shared_ptr<SMotionConfidenceSet>  motionCfd;/* OPTIONAL */
} SFullPositionVector;

typedef enum GNSSstatus {
	GNSSstatus_unavailable	= 0,
	GNSSstatus_isHealthy	= 1,
	GNSSstatus_isMonitored	= 2,
	GNSSstatus_baseStationType	= 3,
	GNSSstatus_aPDOPofUnder5	= 4,
	GNSSstatus_inViewOfUnder5	= 5,
	GNSSstatus_localCorrectionsPresent	= 6,
	GNSSstatus_networkCorrectionsPresent	= 7
} EGNSSstatus;

typedef struct PathHistoryPoint {
	double x;
    double y;
    uint64_t timeOffset;
	std::shared_ptr<double> speed;
    std::shared_ptr<SPositionConfidenceSet> posAccuracy;
    std::shared_ptr<uint8_t> heading;
} SPathHistoryPoint;

typedef struct PathHistory {
    std::shared_ptr<SFullPositionVector>  initialPosition;
	std::shared_ptr<EGNSSstatus> currGNSSstatus	/* OPTIONAL */;
	std::vector<SPathHistoryPoint> crumbData;
} SPathHistory;

typedef enum DriveBehavior {
	DriveBehavior_goStraightForward	= 0,
	DriveBehavior_laneChangingToLeft	= 1,
	DriveBehavior_laneChangingToRight	= 2,
	DriveBehavior_rampIn	= 3,
	DriveBehavior_rampOut	= 4,
	DriveBehavior_intersectionStraightThrough	= 5,
	DriveBehavior_intersectionTurnLeft	= 6,
	DriveBehavior_intersectionTurnRight	= 7,
	DriveBehavior_intersectionUTurn	= 8,
	DriveBehavior_stop_and_go	= 9,
	DriveBehavior_stop	= 10,
	DriveBehavior_slow_down	= 11,
	DriveBehavior_speed_up	= 12,
	DriveBehavior_parking	= 13
} EDriveBehavior;
//
typedef struct NodeReferenceID {
	std::shared_ptr<uint64_t> region;
	uint64_t id;
} SNodeReferenceID;

typedef struct ReferenceLink {
	SNodeReferenceID upstreamNodeId;
	SNodeReferenceID downstreamNodeId;
	std::shared_ptr<uint16_t> referenceLanes	/* OPTIONAL */;
} SReferenceLink;

typedef struct ReferenceLinkList {
	std::vector<SReferenceLink> list;
} SReferenceLinkList;

typedef enum AccConfidence {
	AccConfidence_unavailable	= 0,
	AccConfidence_prec100deg	= 1,
	AccConfidence_prec10deg	= 2,
	AccConfidence_prec5deg	= 3,
	AccConfidence_prec1deg	= 4,
	AccConfidence_prec0_1deg	= 5,
	AccConfidence_prec0_05deg	= 6,
	AccConfidence_prec0_01deg	= 7
} EAccConfidence;

typedef enum AngularVConfidence {
	AngularVConfidence_unavailable	= 0,
	AngularVConfidence_prec100deg	= 1,
	AngularVConfidence_prec10deg	= 2,
	AngularVConfidence_prec5deg	= 3,
	AngularVConfidence_prec1deg	= 4,
	AngularVConfidence_prec0_1deg	= 5,
	AngularVConfidence_prec0_05deg	= 6,
	AngularVConfidence_prec0_01deg	= 7
} EAngularVConfidence;

typedef struct AccSet4WayConfidence {
	EAccConfidence lonAccConfidence;
	EAccConfidence latAccConfidence;
	EAccConfidence vertAccConfidence;
	EAngularVConfidence yawRateCon;
} SAccSet4WayConfidence;

typedef struct PathPlanningPoint {
	std::shared_ptr<SReferenceLink> posInMap;                // OPTIONAL, Lane and Link location related to MAP
	SPosition3D	 pos;                                        // Target location in the path
	std::shared_ptr<PositionConfidenceSet> posAccuracy	/* OPTIONAL */;
	std::shared_ptr<double> speed	/* OPTIONAL */;
	std::shared_ptr<SpeedConfidence> speedCfd	/* OPTIONAL */;
	std::shared_ptr<double> heading	/* OPTIONAL */;
	std::shared_ptr<EHeadingConfidence> headingCfd	/* OPTIONAL */;
	std::shared_ptr<AccelerationSet4Way> accelSet	/* OPTIONAL */;
	std::shared_ptr<AccSet4WayConfidence> acc4WayConfidence	/* OPTIONAL */;
	std::shared_ptr<TimeOffset_t> estimatedTime	/* OPTIONAL */;
	std::shared_ptr<Confidence_t> timeConfidence	/* OPTIONAL */;
} SPathPlanningPoint;

typedef struct PathPlanning {
	std::vector<PathPlanningPoint> list;
} SPathPlanning;

typedef enum ExteriorLights {
	ExteriorLights_lowBeamHeadlightsOn	= 0,
	ExteriorLights_highBeamHeadlightsOn	= 1,
	ExteriorLights_leftTurnSignalOn	= 2,
	ExteriorLights_rightTurnSignalOn	= 3,
	ExteriorLights_hazardSignalOn	= 4,
	ExteriorLights_automaticLightControlOn	= 5,
	ExteriorLights_daytimeRunningLightsOn	= 6,
	ExteriorLights_fogLightOn	= 7,
	ExteriorLights_parkingLightsOn	= 8
} EExteriorLights;

//
typedef uint64_t Radius;

typedef struct PathPointList {
	std::vector<SPosition3D> list;
} SPathPointList;

typedef struct ReferencePath {
	SPathPointList activePath;
	Radius pathRadius;
} SReferencePath;

typedef struct ReferencePathList {
	std::vector<struct ReferencePath> list;
} SReferencePathList;


}
#endif