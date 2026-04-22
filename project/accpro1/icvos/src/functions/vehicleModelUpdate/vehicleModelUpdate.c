
#include "accpro1.h"
void vehicleModelUpdate(
    double accel,
    double dt,
    VehicleState* state
)
{
    // 读取旧状态
    double x0 = state->x;
    double v0 = state->v;
    double a0 = state->a;

    // ---------- 更新位置 ----------
    double x1 = x0 + v0 * dt + 0.5 * accel * dt * dt;

    // ---------- 更新速度 ----------
    double v1 = v0 + accel * dt;

    // ---------- 更新加速度 ----------
    double a1 = accel;

    // ---------- 写回状态 ----------
    state->x = x1;
    state->v = v1;
    state->a = a1;
}