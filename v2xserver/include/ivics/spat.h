#ifndef	_SPAT_H_
#define	_SPAT_H_

#include <stdint.h>
#include <string>
#include <memory>
#include <vector>
#include "common.h"
namespace Ivics
{
typedef enum TimeChangeDetails_PR {
	TimeChangeDetails_PR_NOTHING,	/* No components present */
	TimeChangeDetails_PR_counting,
	TimeChangeDetails_PR_utcTiming
	/* Extensions may appear below */
} ETimeChangeDetails_PR;

typedef struct TimeCountingDown {
	uint64_t startTime; //unit:sec
	std::shared_ptr<uint64_t> minEndTime	/* OPTIONAL */;
	std::shared_ptr<uint64_t> maxEndTime	/* OPTIONAL */;
	uint64_t	 likelyEndTime;
	std::shared_ptr<Confidence_t> timeConfidence	/* OPTIONAL */;
	std::shared_ptr<uint64_t> nextStartTime	/* OPTIONAL */;
	std::shared_ptr<uint64_t> nextDuration	/* OPTIONAL */;
} STimeCountingDown;

typedef struct UTCTiming {
	uint64_t	 startUTCTime;
	std::shared_ptr<uint64_t> minEndUTCTime	/* OPTIONAL */;
	std::shared_ptr<uint64_t> maxEndUTCTime	/* OPTIONAL */;
	uint64_t	 likelyEndUTCTime;
	std::shared_ptr<Confidence_t> timeConfidence	/* OPTIONAL */;
	std::shared_ptr<uint64_t> nextStartUTCTime	/* OPTIONAL */;
	std::shared_ptr<uint64_t> nextEndUTCTime	/* OPTIONAL */;
} SUTCTiming;

/* TimeChangeDetails */
typedef struct TimeChangeDetails {
	ETimeChangeDetails_PR present;
	std::shared_ptr<STimeCountingDown> counting;
	std::shared_ptr<SUTCTiming> utcTiming;
} STimeChangeDetails;

typedef enum LightState {
	LightState_unavailable	= 0,
	LightState_dark	= 1,
	LightState_flashing_red	= 2,
	LightState_red	= 3,
	LightState_flashing_green	= 4,
	LightState_permissive_green	= 5,
	LightState_protected_green	= 6,
	LightState_yellow	= 7,
	LightState_flashing_yellow	= 8
} ELightState;

typedef struct PhaseState {
	ELightState light;
	std::shared_ptr<STimeChangeDetails> timing	/* OPTIONAL */;
} PhaseState_t;

typedef struct PhaseStateList {
	std::vector<struct PhaseState> list;
} SPhaseStateList;

// 定义信号灯相位，一个相位包括一个相位 ID 以及一个相位状态列表
typedef struct Phase {
	long id;
	SPhaseStateList phaseStates;
} SPhase;

typedef struct PhaseList {
	std::vector<struct Phase> list;
} SPhaseList;

// 路口信号机的工作状态指示。用于 SPAT 消息中。
typedef struct IntersectionStatusObject{
	bool manualControlIsEnabled:1;                   // 是否允许手动控制
	bool stopTimeIsActivated:1;                      // And all counting/timing has stopped
	bool failureFlash:1;                             // Above to be used for any detected hardware failures, conflict monitor as well as for police flash
	bool preemptIsActive:1;            
	bool signalPriorityIsActive:1;                   // Additional states
	bool fixedTimeOperation:1;                       // Schedule of signals is based on time only, (i.e. the state can be calculated)
	bool trafficDependentOperation:1;                // 根据交通流量自动信号配时
	bool standbyOperation:1;                         // 黄闪
	bool failureMode:1;                              // Controller has a problem or failure in operation
	bool off:1;                                      // Controller is switched off 
	bool recentMAPmessageUpdate:1;                   // Map revision with content changes
	bool recentChangeInMAPassignedLanesIDsUsed:1;    // Change in MAP's assigned lanes used; Changes in the active lane list description
	bool noValidMAPisAvailableAtThisTime:1;          // MAP (and various lanes indexes) not available
	bool noValidSPATisAvailableAtThisTime:1;         // SPAT system is not working at this time
	bool reserved1:1;
	bool reserved2:2;
}SIntersectionStatusObject;

// 定义一个路口信号灯的属性和当前状态
typedef struct IntersectionState {
	std::string intersectionId;                       // 路口ID
	IntersectionStatusObject status;                  // 信号灯状态
	std::shared_ptr<u_int64_t> moy;                   // minute of the year
	std::shared_ptr<u_int64_t> timeStamp;             // 暂时无效
	std::shared_ptr<TimeConfidence> timeConfidence;   // 暂时无效
	SPhaseList phases;
} SIntersectionState;

typedef struct IntersectionStateList {
	std::vector<struct IntersectionState> list;
} SIntersectionStateList;

// Signal Phase and Timing Message（信号灯信息）
typedef struct SPAT
{
    u_int8_t msgCnt;
	std::shared_ptr<u_int64_t> moy;           // minute of the year
    std::string name;                         // human readable name for this collection, to be used only in debug mode
    std::shared_ptr<u_int64_t> timeStamp;     // Time stamp when this message is formed in milliseconds
    SIntersectionStateList intersections;     // sets of SPAT data (one per intersection)
} SPAT;




}//namespace ivics




#endif	/* _BasicSafetyMessage_H_ */