#ifndef	_SensorSharingMsg_H_
#define	_SensorSharingMsg_H_

#include <stdint.h>
#include <string>
#include <memory>
#include <vector>
#include "common.h"
#include "rsi.h"

namespace Ivics
{
//EquipmentType
typedef enum EquipmentType {
	EquipmentType_unknown	= 0,
	EquipmentType_rsu	= 1,
	EquipmentType_obu	= 2,
	EquipmentType_vru	= 3
} EEquipmentType;

//DetectedRegion
typedef struct Polygon {
	std::vector<SPosition3D> list;
} SPolygon;

typedef struct DetectedRegion {
	std::vector<SPolygon> list;
} SDetectedRegion;

//DetectedPTCList
typedef enum ParticipantType {
	ParticipantType_unknown	= 0,
	ParticipantType_motor	= 1,
	ParticipantType_non_motor	= 2,
	ParticipantType_pedestrian	= 3,
	ParticipantType_rsu	= 4
} EParticipantType;

typedef enum SourceType {
	SourceType_unknown	= 0,
	SourceType_selfinfo	= 1,
	SourceType_v2x	= 2,
	SourceType_video	= 3,
	SourceType_microwaveRadar	= 4,
	SourceType_loop	= 5,
	SourceType_lidar	= 6,
	SourceType_integrated	= 7
} ESourceType;

typedef struct ParticipantData {
	EParticipantType ptcType;
	long ptcId;
	ESourceType source;
	std::shared_ptr<std::string> id;
	uint64_t secMark;
	SPosition3D pos; //unit:m
	SPositionConfidenceSet posConfidence;
	std::shared_ptr<ETransmissionState>	transmission	/* OPTIONAL */;
	double speed; // unit:m/s
	double heading; //unit:radian
	std::shared_ptr<double> angle;
	std::shared_ptr<SMotionConfidenceSet> motionCfd;	/* OPTIONAL */
	std::shared_ptr<SAccelerationSet4Way> accelSet;
	struct VehicleSize size;
	std::shared_ptr<VehicleClassification> vehicleClass;
} SParticipantData;

typedef enum SizeValueConfidence {
	SizeValueConfidence_unavailable	= 0,
	SizeValueConfidence_size_100_00	= 1,
	SizeValueConfidence_size_050_00	= 2,
	SizeValueConfidence_size_020_00	= 3,
	SizeValueConfidence_size_010_00	= 4,
	SizeValueConfidence_size_005_00	= 5,
	SizeValueConfidence_size_002_00	= 6,
	SizeValueConfidence_size_001_00	= 7,
	SizeValueConfidence_size_000_50	= 8,
	SizeValueConfidence_size_000_20	= 9,
	SizeValueConfidence_size_000_10	= 10,
	SizeValueConfidence_size_000_05	= 11,
	SizeValueConfidence_size_000_02	= 12,
	SizeValueConfidence_size_000_01	= 13
} ESizeValueConfidence;

typedef struct ObjectSizeConfidence {
	ESizeValueConfidence widthConf;
	ESizeValueConfidence lengthConf;
	std::shared_ptr<ESizeValueConfidence> heightConf	/* OPTIONAL */;
} SObjectSizeConfidence;

typedef enum DetectedPTCType {
	DetectedPTCType_unknown	= 0,
	DetectedPTCType_unknown_movable	= 1,
	DetectedPTCType_unknown_unmovable	= 2,
	DetectedPTCType_car	= 3,
	DetectedPTCType_van	= 4,
	DetectedPTCType_truck	= 5,
	DetectedPTCType_bus	= 6,
	DetectedPTCType_cyclist	= 7,
	DetectedPTCType_motorcyclist	= 8,
	DetectedPTCType_tricyclist	= 9,
	DetectedPTCType_pedestrian	= 10
} EDetectedPTCType;



typedef struct Planning {
	std::shared_ptr<uint16_t> duration;
	std::shared_ptr<Confidence_t> planConfidence	/* OPTIONAL */;
	std::shared_ptr<EDriveBehavior> drivingBehavior	/* OPTIONAL */;
	std::shared_ptr<PathPlanning> pathPlanning	/* OPTIONAL */;
} SPlanning;

typedef struct PlanningList {
	std::vector<SPlanning> list;
} PlanningList;

typedef enum type_relatedExt_PR {
	type_relatedExt_PR_NOTHING,	/* No components present */
	type_relatedExt_PR_motorExt,
	type_relatedExt_PR_non_motorExt
} ETypeRelatedExtPR;

typedef struct Attitude {
	long pitch;
	long roll;
	long yaw;
} SAttitude;

typedef struct AttitudeConfidence {
	EHeadingConfidence	 pitchConfidence;
	EHeadingConfidence	 rollRateConfidence;
	EHeadingConfidence	 yawRate;
} SAttitudeConfidence;

typedef struct AngularVelocity {
	long pitchRate;
	long rollRate;
	long yawRate;
} SAngularVelocity;

typedef struct AngularVelocityConfidence {
	EAngularVConfidence pitchRate;
	EAngularVConfidence rollRate;
	EAngularVConfidence yawRate;
} SAngularVelocityConfidence;

typedef struct MotorDataExtension {
	std::shared_ptr<EExteriorLights> lights	/* OPTIONAL */;
	std::shared_ptr<SAttitude> vehAttitude	/* OPTIONAL */;
	std::shared_ptr<SAttitudeConfidence> vehAttitudeConfidence	/* OPTIONAL */;
	std::shared_ptr<SAngularVelocity> vehAngVel	/* OPTIONAL */;
	std::shared_ptr<SAngularVelocityConfidence> vehAngVelConfidence	/* OPTIONAL */;
} SMotorDataExtension;

typedef enum PersonalDeviceUserType {
	PersonalDeviceUserType_unavailable	= 0,
	PersonalDeviceUserType_aPEDESTRIAN	= 1,
	PersonalDeviceUserType_aPEDALCYCLIST	= 2,
	PersonalDeviceUserType_aROADWORKER	= 3,
	PersonalDeviceUserType_anANIMAL	= 4
} EPersonalDeviceUserType;

typedef enum PropelledInformation_PR {
	PropelledInformation_PR_NOTHING,	/* No components present */
	PropelledInformation_PR_human,
	PropelledInformation_PR_animal,
	PropelledInformation_PR_motor
} EPropelledInformation_PR;

typedef enum HumanPropelledType {
	HumanPropelledType_unavailable	= 0,
	HumanPropelledType_otherTypes	= 1,
	HumanPropelledType_onFoot	= 2,
	HumanPropelledType_skateboard	= 3,
	HumanPropelledType_pushOrKickScooter	= 4,
	HumanPropelledType_wheelchair	= 5
} EHumanPropelledType;

typedef enum AnimalPropelledType {
	AnimalPropelledType_unavailable	= 0,
	AnimalPropelledType_otherTypes	= 1,
	AnimalPropelledType_animalMounted	= 2,
	AnimalPropelledType_animalDrawnCarriage	= 3
} EAnimalPropelledType;

typedef enum MotorizedPropelledType {
	MotorizedPropelledType_unavailable	= 0,
	MotorizedPropelledType_otherTypes	= 1,
	MotorizedPropelledType_wheelChair	= 2,
	MotorizedPropelledType_bicycle	= 3,
	MotorizedPropelledType_scooter	= 4,
	MotorizedPropelledType_selfBalancingDevice	= 5
} EMotorizedPropelledType;

typedef struct PropelledInformation {
	EPropelledInformation_PR present;
	union UPropelledInformation {
		EHumanPropelledType human;
		EAnimalPropelledType animal;
		EMotorizedPropelledType motor;
	} choice;
} SPropelledInformation;

typedef enum NumberOfParticipantsInCluster {
	NumberOfParticipantsInCluster_unavailable	= 0,
	NumberOfParticipantsInCluster_small	= 1,
	NumberOfParticipantsInCluster_medium	= 2,
	NumberOfParticipantsInCluster_large	= 3
} ENumberOfParticipantsInCluster;

typedef enum Attachment {
	Attachment_unavailable	= 0,
	Attachment_stroller	= 1,
	Attachment_bicycleTrailer	= 2,
	Attachment_cart	= 3,
	Attachment_wheelchair	= 4,
	Attachment_otherWalkAssistAttachments	= 5,
	Attachment_pet	= 6
} EAttachment;

typedef struct PersonalExtensions {
	std::shared_ptr<uint16_t> useState	/* OPTIONAL */;
	std::shared_ptr<uint8_t> assistType	/* OPTIONAL */;
} SPersonalExtensions;

typedef enum RoadWorkerType {
	RoadWorkerType_unavailable	= 0,
	RoadWorkerType_trafficPolice	= 1,
	RoadWorkerType_constructionPersonnel	= 2,
	RoadWorkerType_policeOfficers	= 3,
	RoadWorkerType_trafficControlPersons	= 4,
	RoadWorkerType_railroadCrossingGuards	= 5,
	RoadWorkerType_emergencyOrganizationPersonnel	= 6
} ERoadWorkerType;

typedef struct RoadWorkerExtensions {
	std::shared_ptr<ERoadWorkerType> workerType	/* OPTIONAL */;
	std::shared_ptr<uint8_t> activityType	/* OPTIONAL */;
} SRoadWorkerExtensions;

typedef enum PersonalCrossing {
	PersonalCrossing_unavailable	= 0,
	PersonalCrossing_request	= 1,
	PersonalCrossing_crossing	= 2,
	PersonalCrossing_finish	= 3
} EPersonalCrossing;

typedef struct PersonalRequest {
	std::shared_ptr<EPersonalCrossing> crossing	/* OPTIONAL */;
} SPersonalRequest;

typedef struct Non_motorData {
	EPersonalDeviceUserType basicType;
	std::shared_ptr<SPropelledInformation> propulsion	/* OPTIONAL */;
	std::shared_ptr<ENumberOfParticipantsInCluster> clusterSize	/* OPTIONAL */;
	std::shared_ptr<EAttachment> attachment	/* OPTIONAL */;
	std::shared_ptr<SPersonalExtensions> personalExt	/* OPTIONAL */;
	std::shared_ptr<SRoadWorkerExtensions> roadWorkerExt	/* OPTIONAL */;
	std::shared_ptr<PersonalRequest> personalReq	/* OPTIONAL */;
} Non_motorData_t;

typedef struct Non_motorDataExtension {
	long overallRadius;
	Non_motorData_t	 non_motorData;
} SNonMotorDataExtension;

typedef struct type_relatedExt {
	ETypeRelatedExtPR present;
	union UtypeRelatedExt {
		SMotorDataExtension motorExt;
		SNonMotorDataExtension non_motorExt;
	} choice;
}STypeRelatedExt;

typedef struct DetectedPTCData {
	SParticipantData ptc;
	std::shared_ptr<SObjectSizeConfidence> objSizeConfidence	/* OPTIONAL */;
	std::shared_ptr<EDetectedPTCType> detectedPTCType	/* OPTIONAL */;
	std::shared_ptr<uint8_t> typeConfidence	/* OPTIONAL */;
	std::shared_ptr<SAccSet4WayConfidence> acc4WayConfidence	/* OPTIONAL */;
	std::shared_ptr<uint64_t> statusDuration	/* OPTIONAL */;
	std::shared_ptr<SPathHistory> pathHistory	/* OPTIONAL */;
	std::shared_ptr<PlanningList> planningList	/* OPTIONAL */;
	std::shared_ptr<uint64_t> tracking	/* OPTIONAL */;
	std::shared_ptr<SPolygon> polygon;
	std::shared_ptr<STypeRelatedExt> type_relatedExt;
} DetectedPTCData_t;

typedef struct DetectedPTCList {
	std::vector<struct DetectedPTCData> list;
} DetectedPTCList_t;

//
typedef enum ObstacleType {
	ObstacleType_unknown	= 0,
	ObstacleType_rockfall	= 1,
	ObstacleType_landslide	= 2,
	ObstacleType_animal_intrusion	= 3,
	ObstacleType_liquid_spill	= 4,
	ObstacleType_goods_scattered	= 5,
	ObstacleType_trafficcone	= 6,
	ObstacleType_safety_triangle	= 7,
	ObstacleType_traffic_roadblock	= 8,
	ObstacleType_inspection_shaft_without_cover	= 9,
	ObstacleType_unknown_fragments	= 10,
	ObstacleType_unknown_hard_object	= 11,
	ObstacleType_unknown_soft_object	= 12
} EObstacleType;

typedef struct ObjectSize {
	double width;
	double length;
	std::shared_ptr<double> height	/* OPTIONAL */;
} ObjectSize_t;

typedef struct DetectedObstacleData {
	EObstacleType obsType;
	std::shared_ptr<Confidence_t> objTypeConfidence	/* OPTIONAL */;
	long obsId;
	ESourceType source;
	uint64_t secMark;
	SPosition3D pos;
	SPositionConfidenceSet posConfidence;
	double speed;
	std::shared_ptr<ESpeedConfidence> speedCfd	/* OPTIONAL */;
	double heading;
	std::shared_ptr<EHeadingConfidence> headingCfd	/* OPTIONAL */;
	std::shared_ptr<double> verSpeed	/* OPTIONAL */;
	std::shared_ptr<ESpeedConfidence> verSpeedConfidence	/* OPTIONAL */;
	std::shared_ptr<struct AccelerationSet4Way> accelSet	/* OPTIONAL */;
	ObjectSize_t	 size;
	std::shared_ptr<struct ObjectSizeConfidence> objSizeConfidence	/* OPTIONAL */;
	std::shared_ptr<long> tracking	/* OPTIONAL */;
	std::shared_ptr<struct Polygon> polygon	/* OPTIONAL */;
} DetectedObstacleData_t;

typedef struct DetectedObstacleList {
	std::vector<struct DetectedObstacleData> list;
} SDetectedObstacleList;

// 感知共享消息（Sensor Sharing Message）
typedef struct SensorSharingMsg {
	u_int8_t msgCnt;
	std::string id;                    // temperary vehicle ID
	EEquipmentType equipmentType;      // RSU
	uint64_t secMark;                  // 雷达启动时间，单位毫秒 
	SPosition3D sensorPos;             // 传感器位置，始终为空

	std::shared_ptr<struct DetectedRegion> detectedRegion;  // 始终为空
	std::shared_ptr<struct DetectedPTCList> participants;   // 交通参与者
	std::shared_ptr<struct DetectedObstacleList> obstacles; // 障碍物
	std::shared_ptr<struct RTEList> rtes;                   // 始终为空
} SSM;



}//namespace ivics




#endif	/* _BasicSafetyMessage_H_ */