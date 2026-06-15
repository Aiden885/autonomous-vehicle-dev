#ifndef MATRIXOPS_H
#define MATRIXOPS_H

#ifdef __cplusplus
extern "C" {
#endif

#define MATRIX_MAX_DIM 10

/**
 * @brief 固定上限的矩阵结构体，用于LQR/MPC控制器计算
 * @en_name Matrix
 * @cn_name 矩阵
 * @tag STRUCT
 * @field name=Data, type=Array<double,[MATRIX_MAX_DIM,MATRIX_MAX_DIM]>, unit=1, desc=矩阵数据（行主序）
 * @field name=Rows, type=int, unit=1, desc=有效行数
 * @field name=Cols, type=int, unit=1, desc=有效列数
 * @version 1.0
 * @date 2026-03-27
 * @author aiden
 */
typedef struct {
    double Data[MATRIX_MAX_DIM][MATRIX_MAX_DIM];
    int    Rows;
    int    Cols;
} Matrix;

void matrixInit(Matrix *mat, int rows, int cols);
void matrixSetIdentity(Matrix *mat);
void matrixCopy(const Matrix *src, Matrix *dst);
void matrixAdd(const Matrix *a, const Matrix *b, Matrix *result);
void matrixSub(const Matrix *a, const Matrix *b, Matrix *result);
void matrixMul(const Matrix *a, const Matrix *b, Matrix *result);
void matrixScale(const Matrix *a, double scalar, Matrix *result);
void matrixTranspose(const Matrix *a, Matrix *result);
void matrixInverse(const Matrix *a, Matrix *result);

#ifdef __cplusplus
}
#endif
#endif /* MATRIXOPS_H */
