#include "matrixOps.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>

#define MATRIX_EPSILON 1e-12

static int matrixClipDim(int value);
static int matrixMinInt(int left, int right);
static void matrixZeroData(Matrix *mat);
static void matrixInverseInitAugmented(const Matrix *a,
    double augmented[MATRIX_MAX_DIM][2 * MATRIX_MAX_DIM], int size);
static void matrixInverseSwapRows(double augmented[MATRIX_MAX_DIM][2 * MATRIX_MAX_DIM],
    int rowA, int rowB, int totalCols);
static void matrixInverseScaleRow(double augmented[MATRIX_MAX_DIM][2 * MATRIX_MAX_DIM],
    int row, int totalCols, double divisor);
static void matrixInverseEliminateRow(double augmented[MATRIX_MAX_DIM][2 * MATRIX_MAX_DIM],
    int pivotRow, int targetRow, int pivotCol, int totalCols, double factor);
static void matrixInverseExtract(const double augmented[MATRIX_MAX_DIM][2 * MATRIX_MAX_DIM],
    Matrix *result, int size);

/**
 * @brief 将维度值截断到[0, MATRIX_MAX_DIM]合法范围
 * @en_name matrixClipDim
 * @cn_name 维度截断
 * @type widget
 * @param[IN] int value 输入维度值
 * @retval int 截断后的维度值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static int matrixClipDim(int value)
{
    int result;
    bool isNeg;
    bool isTooLarge;

    result = value;
    isNeg = (value < 0);
    isTooLarge = (value > MATRIX_MAX_DIM);

    if (isNeg) {
        result = 0;
    }

    if (isTooLarge) {
        result = MATRIX_MAX_DIM;
    }

    return result;
}

/**
 * @brief 返回两个整数中的较小值
 * @en_name matrixMinInt
 * @cn_name 取最小整数
 * @type widget
 * @param[IN] int left 左侧输入值
 * @param[IN] int right 右侧输入值
 * @retval int 较小值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static int matrixMinInt(int left, int right)
{
    int result;
    bool leftSmaller;

    result = right;
    leftSmaller = (left < right);

    if (leftSmaller) {
        result = left;
    }

    return result;
}

/**
 * @brief 将矩阵数据区全部清零
 * @en_name matrixZeroData
 * @cn_name 矩阵数据清零
 * @type widget
 * @param[OUT] Matrix *mat 目标矩阵
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static void matrixZeroData(Matrix *mat)
{
    bool hasMatrix;

    hasMatrix = (mat != NULL);

    if (hasMatrix) {
        memset(mat->Data, 0, sizeof(mat->Data));
    }
}

/**
 * @brief 初始化增广矩阵[A|I]
 * @en_name matrixInverseInitAugmented
 * @cn_name 初始化增广矩阵
 * @type widget
 * @param[IN] const Matrix *a 输入方阵
 * @param[IN] int size 方阵阶数
 * @param[OUT] double augmented[][] 初始化后的增广矩阵
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static void matrixInverseInitAugmented(const Matrix *a,
    double augmented[MATRIX_MAX_DIM][2 * MATRIX_MAX_DIM], int size)
{
    int rowIndex;
    int colIndex;

    rowIndex = 0;
    for (rowIndex; rowIndex < size; rowIndex = rowIndex + 1) {
        colIndex = 0;
        for (colIndex; colIndex < (2 * size); colIndex = colIndex + 1) {
            augmented[rowIndex][colIndex] = 0.0;
        }
    }

    rowIndex = 0;
    for (rowIndex; rowIndex < size; rowIndex = rowIndex + 1) {
        colIndex = 0;
        for (colIndex; colIndex < size; colIndex = colIndex + 1) {
            augmented[rowIndex][colIndex] = a->Data[rowIndex][colIndex];
        }
        augmented[rowIndex][size + rowIndex] = 1.0;
    }
}

/**
 * @brief 交换增广矩阵两行
 * @en_name matrixInverseSwapRows
 * @cn_name 增广矩阵行交换
 * @type widget
 * @param[IN] int rowA 第一行索引
 * @param[IN] int rowB 第二行索引
 * @param[IN] int totalCols 总列数
 * @param[OUT] double augmented[][] 增广矩阵
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static void matrixInverseSwapRows(double augmented[MATRIX_MAX_DIM][2 * MATRIX_MAX_DIM],
    int rowA, int rowB, int totalCols)
{
    int colIndex;
    double tempValue;

    colIndex = 0;
    for (colIndex; colIndex < totalCols; colIndex = colIndex + 1) {
        tempValue = augmented[rowA][colIndex];
        augmented[rowA][colIndex] = augmented[rowB][colIndex];
        augmented[rowB][colIndex] = tempValue;
    }
}

/**
 * @brief 对增广矩阵指定行除以divisor进行归一化
 * @en_name matrixInverseScaleRow
 * @cn_name 增广矩阵行归一化
 * @type widget
 * @param[IN] int row 行索引
 * @param[IN] int totalCols 总列数
 * @param[IN] double divisor 归一化除数
 * @param[OUT] double augmented[][] 增广矩阵
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static void matrixInverseScaleRow(double augmented[MATRIX_MAX_DIM][2 * MATRIX_MAX_DIM],
    int row, int totalCols, double divisor)
{
    int colIndex;

    colIndex = 0;
    for (colIndex; colIndex < totalCols; colIndex = colIndex + 1) {
        augmented[row][colIndex] = augmented[row][colIndex] / divisor;
    }
}

/**
 * @brief 用主元行对目标行进行消元
 * @en_name matrixInverseEliminateRow
 * @cn_name 增广矩阵行消元
 * @type widget
 * @param[IN] int pivotRow 主元行索引
 * @param[IN] int targetRow 目标行索引
 * @param[IN] int pivotCol 主元列索引
 * @param[IN] int totalCols 总列数
 * @param[IN] double factor 消元因子
 * @param[OUT] double augmented[][] 增广矩阵
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static void matrixInverseEliminateRow(double augmented[MATRIX_MAX_DIM][2 * MATRIX_MAX_DIM],
    int pivotRow, int targetRow, int pivotCol, int totalCols, double factor)
{
    int colIndex;

    colIndex = 0;
    for (colIndex; colIndex < totalCols; colIndex = colIndex + 1) {
        augmented[targetRow][colIndex] =
            augmented[targetRow][colIndex] - factor * augmented[pivotRow][colIndex];
    }
    augmented[targetRow][pivotCol] = 0.0;
}

/**
 * @brief 从增广矩阵右半部分提取逆矩阵
 * @en_name matrixInverseExtract
 * @cn_name 提取逆矩阵
 * @type widget
 * @param[IN] double augmented[][] 完成消元的增广矩阵
 * @param[IN] int size 方阵阶数
 * @param[OUT] Matrix *result 逆矩阵
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static void matrixInverseExtract(const double augmented[MATRIX_MAX_DIM][2 * MATRIX_MAX_DIM],
    Matrix *result, int size)
{
    int rowIndex;
    int colIndex;

    matrixInit(result, size, size);

    rowIndex = 0;
    for (rowIndex; rowIndex < size; rowIndex = rowIndex + 1) {
        colIndex = 0;
        for (colIndex; colIndex < size; colIndex = colIndex + 1) {
            result->Data[rowIndex][colIndex] = augmented[rowIndex][size + colIndex];
        }
    }
}

/**
 * @brief 初始化矩阵，设置行列数并清零数据区
 * @en_name matrixInit
 * @cn_name 矩阵初始化
 * @type widget
 * @param[IN] int rows 行数
 * @param[IN] int cols 列数
 * @param[OUT] Matrix *mat 初始化后的矩阵
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void matrixInit(Matrix *mat, int rows, int cols)
{
    bool hasMatrix;

    hasMatrix = (mat != NULL);

    if (hasMatrix) {
        matrixZeroData(mat);
        mat->Rows = matrixClipDim(rows);
        mat->Cols = matrixClipDim(cols);
    }
}

/**
 * @brief 将矩阵设置为单位矩阵
 * @en_name matrixSetIdentity
 * @cn_name 设置单位矩阵
 * @type widget
 * @param[OUT] Matrix *mat 输出单位矩阵
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void matrixSetIdentity(Matrix *mat)
{
    int rows;
    int cols;
    int diagSize;
    int index;
    bool hasMatrix;

    hasMatrix = (mat != NULL);

    if (hasMatrix) {
        rows = matrixClipDim(mat->Rows);
        cols = matrixClipDim(mat->Cols);
        diagSize = matrixMinInt(rows, cols);
        matrixInit(mat, rows, cols);

        index = 0;
        for (index; index < diagSize; index = index + 1) {
            mat->Data[index][index] = 1.0;
        }
    }
}

/**
 * @brief 将源矩阵内容完整复制到目标矩阵
 * @en_name matrixCopy
 * @cn_name 矩阵复制
 * @type widget
 * @param[IN] const Matrix *src 源矩阵
 * @param[OUT] Matrix *dst 目标矩阵
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void matrixCopy(const Matrix *src, Matrix *dst)
{
    bool srcValid;
    bool dstValid;
    bool canCopy;
    bool shouldInitDst;

    srcValid = (src != NULL);
    dstValid = (dst != NULL);
    canCopy = (srcValid && dstValid);
    shouldInitDst = (!canCopy && dstValid);

    if (canCopy) {
        memmove(dst, src, sizeof(Matrix));
        dst->Rows = matrixClipDim(dst->Rows);
        dst->Cols = matrixClipDim(dst->Cols);
    }

    if (shouldInitDst) {
        matrixInit(dst, 0, 0);
    }
}

/**
 * @brief 计算两矩阵对应元素之和，维度不匹配时result置零
 * @en_name matrixAdd
 * @cn_name 矩阵加法
 * @type widget
 * @param[IN] const Matrix *a 输入矩阵A
 * @param[IN] const Matrix *b 输入矩阵B
 * @param[OUT] Matrix *result 加法结果矩阵
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void matrixAdd(const Matrix *a, const Matrix *b, Matrix *result)
{
    Matrix tempResult;
    int rowsA;
    int colsA;
    int rowsB;
    int colsB;
    int rowIndex;
    int colIndex;
    bool hasResult;
    bool hasInputs;
    bool dimsMatch;

    hasResult = (result != NULL);

    if (hasResult) {
        matrixInit(&tempResult, 0, 0);
        hasInputs = ((a != NULL) && (b != NULL));

        if (hasInputs) {
            rowsA = matrixClipDim(a->Rows);
            colsA = matrixClipDim(a->Cols);
            rowsB = matrixClipDim(b->Rows);
            colsB = matrixClipDim(b->Cols);
            dimsMatch = ((rowsA == rowsB) && (colsA == colsB));

            if (dimsMatch) {
                matrixInit(&tempResult, rowsA, colsA);
                rowIndex = 0;
                for (rowIndex; rowIndex < rowsA; rowIndex = rowIndex + 1) {
                    colIndex = 0;
                    for (colIndex; colIndex < colsA; colIndex = colIndex + 1) {
                        tempResult.Data[rowIndex][colIndex] =
                            a->Data[rowIndex][colIndex] + b->Data[rowIndex][colIndex];
                    }
                }
            }
        }

        matrixCopy(&tempResult, result);
    }
}

/**
 * @brief 计算两矩阵对应元素之差，维度不匹配时result置零
 * @en_name matrixSub
 * @cn_name 矩阵减法
 * @type widget
 * @param[IN] const Matrix *a 输入矩阵A
 * @param[IN] const Matrix *b 输入矩阵B
 * @param[OUT] Matrix *result 减法结果矩阵
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void matrixSub(const Matrix *a, const Matrix *b, Matrix *result)
{
    Matrix tempResult;
    int rowsA;
    int colsA;
    int rowsB;
    int colsB;
    int rowIndex;
    int colIndex;
    bool hasResult;
    bool hasInputs;
    bool dimsMatch;

    hasResult = (result != NULL);

    if (hasResult) {
        matrixInit(&tempResult, 0, 0);
        hasInputs = ((a != NULL) && (b != NULL));

        if (hasInputs) {
            rowsA = matrixClipDim(a->Rows);
            colsA = matrixClipDim(a->Cols);
            rowsB = matrixClipDim(b->Rows);
            colsB = matrixClipDim(b->Cols);
            dimsMatch = ((rowsA == rowsB) && (colsA == colsB));

            if (dimsMatch) {
                matrixInit(&tempResult, rowsA, colsA);
                rowIndex = 0;
                for (rowIndex; rowIndex < rowsA; rowIndex = rowIndex + 1) {
                    colIndex = 0;
                    for (colIndex; colIndex < colsA; colIndex = colIndex + 1) {
                        tempResult.Data[rowIndex][colIndex] =
                            a->Data[rowIndex][colIndex] - b->Data[rowIndex][colIndex];
                    }
                }
            }
        }

        matrixCopy(&tempResult, result);
    }
}

/**
 * @brief 计算两矩阵乘积，维度不匹配时result置零
 * @en_name matrixMul
 * @cn_name 矩阵乘法
 * @type widget
 * @param[IN] const Matrix *a 左乘矩阵（M×K）
 * @param[IN] const Matrix *b 右乘矩阵（K×N）
 * @param[OUT] Matrix *result 乘积结果矩阵（M×N）
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void matrixMul(const Matrix *a, const Matrix *b, Matrix *result)
{
    Matrix tempResult;
    int rowsA;
    int colsA;
    int colsB;
    int rowIndex;
    int colIndex;
    int innerIndex;
    double sumValue;
    bool hasResult;
    bool hasInputs;
    bool canMultiply;

    hasResult = (result != NULL);

    if (hasResult) {
        matrixInit(&tempResult, 0, 0);
        hasInputs = ((a != NULL) && (b != NULL));

        if (hasInputs) {
            rowsA = matrixClipDim(a->Rows);
            colsA = matrixClipDim(a->Cols);
            colsB = matrixClipDim(b->Cols);
            canMultiply = (colsA == matrixClipDim(b->Rows));

            if (canMultiply) {
                matrixInit(&tempResult, rowsA, colsB);
                rowIndex = 0;
                for (rowIndex; rowIndex < rowsA; rowIndex = rowIndex + 1) {
                    colIndex = 0;
                    for (colIndex; colIndex < colsB; colIndex = colIndex + 1) {
                        sumValue = 0.0;
                        innerIndex = 0;
                        for (innerIndex; innerIndex < colsA; innerIndex = innerIndex + 1) {
                            sumValue = sumValue +
                                a->Data[rowIndex][innerIndex] * b->Data[innerIndex][colIndex];
                        }
                        tempResult.Data[rowIndex][colIndex] = sumValue;
                    }
                }
            }
        }

        matrixCopy(&tempResult, result);
    }
}

/**
 * @brief 计算矩阵与标量的乘积
 * @en_name matrixScale
 * @cn_name 矩阵标量乘法
 * @type widget
 * @param[IN] const Matrix *a 输入矩阵
 * @param[IN] double scalar 缩放系数
 * @param[OUT] Matrix *result 缩放结果矩阵
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void matrixScale(const Matrix *a, double scalar, Matrix *result)
{
    Matrix tempResult;
    int rows;
    int cols;
    int rowIndex;
    int colIndex;
    bool hasResult;
    bool hasInput;

    hasResult = (result != NULL);

    if (hasResult) {
        matrixInit(&tempResult, 0, 0);
        hasInput = (a != NULL);

        if (hasInput) {
            rows = matrixClipDim(a->Rows);
            cols = matrixClipDim(a->Cols);
            matrixInit(&tempResult, rows, cols);

            rowIndex = 0;
            for (rowIndex; rowIndex < rows; rowIndex = rowIndex + 1) {
                colIndex = 0;
                for (colIndex; colIndex < cols; colIndex = colIndex + 1) {
                    tempResult.Data[rowIndex][colIndex] = a->Data[rowIndex][colIndex] * scalar;
                }
            }
        }

        matrixCopy(&tempResult, result);
    }
}

/**
 * @brief 计算矩阵的转置
 * @en_name matrixTranspose
 * @cn_name 矩阵转置
 * @type widget
 * @param[IN] const Matrix *a 输入矩阵（M×N）
 * @param[OUT] Matrix *result 转置结果矩阵（N×M）
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void matrixTranspose(const Matrix *a, Matrix *result)
{
    Matrix tempResult;
    int rows;
    int cols;
    int rowIndex;
    int colIndex;
    bool hasResult;
    bool hasInput;

    hasResult = (result != NULL);

    if (hasResult) {
        matrixInit(&tempResult, 0, 0);
        hasInput = (a != NULL);

        if (hasInput) {
            rows = matrixClipDim(a->Rows);
            cols = matrixClipDim(a->Cols);
            matrixInit(&tempResult, cols, rows);

            rowIndex = 0;
            for (rowIndex; rowIndex < rows; rowIndex = rowIndex + 1) {
                colIndex = 0;
                for (colIndex; colIndex < cols; colIndex = colIndex + 1) {
                    tempResult.Data[colIndex][rowIndex] = a->Data[rowIndex][colIndex];
                }
            }
        }

        matrixCopy(&tempResult, result);
    }
}

/**
 * @brief 用高斯-约旦消元法求矩阵逆，奇异矩阵时result置零
 * @en_name matrixInverse
 * @cn_name 矩阵求逆
 * @type module
 * @param[IN] const Matrix *a 输入方阵
 * @param[OUT] Matrix *result 逆矩阵，奇异时为零矩阵
 * @retval void 无返回值
 * @granularity composite
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void matrixInverse(const Matrix *a, Matrix *result)
{
    Matrix tempResult;
    double augmented[MATRIX_MAX_DIM][2 * MATRIX_MAX_DIM];
    int size;
    int totalCols;
    int pivotCol;
    int pivotRow;
    int searchRow;
    int targetRow;
    double pivotValue;
    double pivotMagnitude;
    double candidateMagnitude;
    double factor;
    bool hasResult;
    bool hasInput;
    bool isSquare;
    bool inverseSuccess;
    bool shouldProcess;
    bool pivotFound;
    bool shouldReplace;
    bool needSwap;
    bool pivotValid;
    bool isDiffRow;
    bool needsElim;
    bool shouldExtract;

    hasResult = (result != NULL);

    if (hasResult) {
        matrixInit(&tempResult, 0, 0);
        hasInput = (a != NULL);

        if (hasInput) {
            size = matrixClipDim(a->Rows);
            isSquare = ((size == a->Cols) && (size > 0));

            if (isSquare) {
                totalCols = 2 * size;
                matrixInverseInitAugmented(a, augmented, size);
                inverseSuccess = true;

                pivotCol = 0;
                for (pivotCol; pivotCol < size; pivotCol = pivotCol + 1) {
                    shouldProcess = inverseSuccess;
                    if (shouldProcess) {
                        pivotRow = pivotCol;
                        pivotMagnitude = 0.0;
                        pivotFound = false;

                        searchRow = pivotCol;
                        for (searchRow; searchRow < size; searchRow = searchRow + 1) {
                            candidateMagnitude = fabs(augmented[searchRow][pivotCol]);
                            shouldReplace = ((candidateMagnitude > MATRIX_EPSILON) &&
                                (!pivotFound || (candidateMagnitude > pivotMagnitude)));
                            if (shouldReplace) {
                                pivotRow = searchRow;
                                pivotMagnitude = candidateMagnitude;
                                pivotFound = true;
                            }
                        }

                        if (!pivotFound) {
                            inverseSuccess = false;
                        }

                        shouldProcess = (inverseSuccess && pivotFound);
                        if (shouldProcess) {
                            needSwap = (pivotRow != pivotCol);
                            if (needSwap) {
                                matrixInverseSwapRows(augmented, pivotCol, pivotRow, totalCols);
                            }

                            pivotValue = augmented[pivotCol][pivotCol];
                            pivotValid = (fabs(pivotValue) > MATRIX_EPSILON);

                            if (pivotValid) {
                                matrixInverseScaleRow(augmented, pivotCol, totalCols, pivotValue);

                                targetRow = 0;
                                for (targetRow; targetRow < size; targetRow = targetRow + 1) {
                                    isDiffRow = (targetRow != pivotCol);
                                    if (isDiffRow) {
                                        factor = augmented[targetRow][pivotCol];
                                        needsElim = (fabs(factor) > MATRIX_EPSILON);
                                        if (needsElim) {
                                            matrixInverseEliminateRow(augmented, pivotCol,
                                                targetRow, pivotCol, totalCols, factor);
                                        }
                                    }
                                }
                            } else {
                                inverseSuccess = false;
                            }
                        }
                    }
                }

                shouldExtract = inverseSuccess;
                if (shouldExtract) {
                    matrixInverseExtract(augmented, &tempResult, size);
                }
            }
        }

        matrixCopy(&tempResult, result);
    }
}
