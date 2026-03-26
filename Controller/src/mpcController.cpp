#include "mpc_controller_linear.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* =========================
   内部函数：线性化车辆模型
   ========================= */
static void linearModel(double dt, double wheelbase,
                        const double x[STATE_DIM],
                        double A[STATE_DIM][STATE_DIM],
                        double B[STATE_DIM][CONTROL_DIM])
{
    // 步骤1：线性化离散时间模型 x_{k+1} = A x_k + B u_k
    memset(A, 0, sizeof(double)*STATE_DIM*STATE_DIM);
    memset(B, 0, sizeof(double)*STATE_DIM*CONTROL_DIM);

    A[0][0] = 1.0; A[0][1] = dt; A[0][2] = 0.0;
    A[1][1] = 1.0; A[1][2] = 0.0;
    A[2][2] = 1.0;

    B[0][0] = 0.0;     B[0][1] = 0.0;
    B[1][0] = dt;      B[1][1] = 0.0;
    B[2][0] = 0.0;     B[2][1] = dt;
}

/* =========================
   内部函数：构造H和f矩阵
   ========================= */
static void buildQPMatrix(int horizon,
                          double Q[STATE_DIM][STATE_DIM],
                          double R[CONTROL_DIM][CONTROL_DIM],
                          double A[STATE_DIM][STATE_DIM],
                          double B[STATE_DIM][CONTROL_DIM],
                          const double x0[STATE_DIM],
                          double H[MPC_HORIZON_MAX*CONTROL_DIM*MPC_HORIZON_MAX*CONTROL_DIM],
                          double f[MPC_HORIZON_MAX*CONTROL_DIM])
{
    // 步骤2：构造代价函数 H, f
    memset(H, 0, sizeof(double)*MPC_HORIZON_MAX*CONTROL_DIM*MPC_HORIZON_MAX*CONTROL_DIM);
    memset(f, 0, sizeof(double)*MPC_HORIZON_MAX*CONTROL_DIM);

    // 实际应用中需矩阵乘法:
    // H = Gamma^T Q Gamma + R
    // f = Gamma^T Q Phi x0
}

/* =========================
   内部函数：求解QP
   ========================= */
static void solveQP(int n_vars,
                    double* H, double* f,
                    double* lb, double* ub,
                    double* u_opt)
{
    // 步骤3：调用QP求解器
    for(int i=0;i<n_vars;i++)
        u_opt[i] = 0.0; // 暂时占位
}

/* =========================
   内部函数：计算初始状态误差
   ========================= */
static void computeStateError(const State* state,
                              const TrajPoint* ref,
                              double x[STATE_DIM])
{
    // 步骤0：将全局坐标误差转换为Frenet误差
    double dx = state->x - ref->x;
    double dy = state->y - ref->y;
    double yaw_ref = ref->yaw;

    x[0] = -sin(yaw_ref)*dx + cos(yaw_ref)*dy;  // cte
    x[1] = state->yaw - yaw_ref;                // epsi
    x[2] = state->v - ref->v;                   // v_error
}

/* =========================
   MPC主函数
   ========================= */
void mpc_controller_linear(const State* state,
                           const Traj* traj,
                           ControlCmd* cmd,
                           int horizon,
                           double dt,
                           double wheelbase,
                           double w_cte,
                           double w_epsi,
                           double w_v,
                           double w_steer,
                           double w_acc)
{
    // ===== 步骤0：计算初始误差 =====
    double x0[STATE_DIM];
    computeStateError(state, &traj->points[0], x0);

    // ===== 步骤1：线性化车辆模型 =====
    double A[STATE_DIM][STATE_DIM], B[STATE_DIM][CONTROL_DIM];
    linearModel(dt, wheelbase, x0, A, B);

    // ===== 步骤2：构造QP代价函数 =====
    double Q[STATE_DIM][STATE_DIM] = {{w_cte,0,0},{0,w_epsi,0},{0,0,w_v}};
    double R[CONTROL_DIM][CONTROL_DIM] = {{w_steer,0},{0,w_acc}};
    double H[MPC_HORIZON_MAX*CONTROL_DIM*MPC_HORIZON_MAX*CONTROL_DIM];
    double f[MPC_HORIZON_MAX*CONTROL_DIM];
    buildQPMatrix(horizon, Q, R, A, B, x0, H, f);

    // ===== 步骤3：设置约束 =====
    double lb[MPC_HORIZON_MAX*CONTROL_DIM];
    double ub[MPC_HORIZON_MAX*CONTROL_DIM];
    for(int i=0;i<horizon*CONTROL_DIM;i++){
        lb[i] = -1.0; // steer -1rad, accel -1m/s²
        ub[i] = 1.0;  // steer 1rad, accel 1m/s²
    }

    // ===== 步骤4：求解QP =====
    double u_opt[MPC_HORIZON_MAX*CONTROL_DIM];
    solveQP(horizon*CONTROL_DIM, H, f, lb, ub, u_opt);

    // ===== 步骤5：输出控制量 =====
    cmd->steer = u_opt[0];
    cmd->acceleration = u_opt[1];
    cmd->speed = traj->points[0].v;
}