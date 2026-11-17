#ifndef APA_MOCO_ACTUATOR_H
#define APA_MOCO_ACTUATOR_H

#include "baseTypes.h"

/* Shared brake gain and pedal slew also define the prediction build rate. */
#define APA_MOCO_DRIVE_GAIN_MPS2 1.5f
#define APA_MOCO_CREEP_THROTTLE 0.12f
#define APA_MOCO_BRAKE_GAIN_MPS2 6.0f
#define APA_MOCO_BRAKE_RISE_PER_SEC 4.0f
#define APA_MOCO_BRAKE_BUILD_JERK_MPS3 (APA_MOCO_BRAKE_GAIN_MPS2 * APA_MOCO_BRAKE_RISE_PER_SEC)
#define APA_MOCO_LAUNCH_ACCEL_MPS2 0.35f
#define APA_MOCO_LAUNCH_SPEED_MPS 0.15f
#define APA_MOCO_OVERSPEED_TOLERANCE_MPS 0.25f

/* Signed longitudinal quantities use the vehicle's forward axis. */
typedef struct {
  Real VehicleSpeed;        /* Longitudinal speed, m/s. */
  Real VehicleMotionSpeed;  /* Nonnegative 3D speed magnitude, m/s. */
  Real VehicleAcceleration; /* Longitudinal acceleration, m/s2. */
  Real GravityAcceleration; /* Road-plane gravity along forward axis, m/s2. */
  Real TargetSpeed;         /* Requested longitudinal speed, m/s. */
  Real TargetAcceleration;  /* Requested longitudinal acceleration, m/s2. */
  Real SpeedLimit;          /* Positive task speed limit, m/s. */
  Real CycleTime;           /* Accepted physics-frame interval, s. */
  Integer FeedbackGear;     /* Measured APA gear enum. */
  Integer TargetGear;       /* Requested APA gear enum. */
  Boolean Enable;
  Boolean StopReq;
  Boolean GearChangeAllowed; /* Controller permits a stopped gear change. */
} ApaMocoActuatorInput_t;

/* Zero-initialize on a new task; advance once per accepted physics frame. */
typedef struct {
  Real Bias;                 /* Learned acceleration compensation, m/s2. */
  Real Throttle;             /* Previous throttle command, 0..1. */
  Real Brake;                /* Previous brake command, 0..1. */
  Real FilteredAcceleration; /* Acceleration along selected direction, m/s2. */
  Integer Direction;         /* Previous travel direction, -1 or +1; 0 unset. */
  Boolean Overspeed;         /* Absolute-speed fault, latched until task reset. */
  Boolean LaunchHold;        /* Build drive demand before releasing the stopped brake. */
} ApaMocoActuatorState_t;

typedef struct {
  Real Throttle; /* Throttle command, 0..1. */
  Real Brake;    /* Brake command, 0..1. */
  Integer Gear;  /* Admitted APA gear enum. */
  Boolean HandBrake;
  Boolean LaunchHold; /* Brake/traction overlap during stopped launch or rollback arrest. */
  Boolean Overspeed;  /* Latched absolute-speed fault. */
} ApaMocoActuatorOutput_t;

void apaMocoActuate(const ApaMocoActuatorInput_t *input,
                    ApaMocoActuatorState_t *state,
                    ApaMocoActuatorOutput_t *output);

#endif
