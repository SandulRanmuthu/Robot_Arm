#include <Arduino.h>
#include "matrix.h"

void matrixMultiply(
    const float A[3][3],
    const float B[3][3],
    float result[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result[i][j] = 0;

            for (int k = 0; k < 3; k++)
            {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}


void matrixVectorMultiply(
    const float A[3][3],
    const float B[3],
    float result[3])
{
    for (int i = 0; i < 3; i++)
    {
        result[i] = 0;

        for (int j = 0; j < 3; j++)
        {
            result[i] += A[i][j] * B[j];
        }
    }
}


float matrixDeterminant(const float A[3][3])
{
    return
        A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1])
      - A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0])
      + A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);
}


bool matrixInverse(
    const float A[3][3],
    float result[3][3])
{
    float det = matrixDeterminant(A);

    // Check for singular matrix
    if (fabs(det) < 1e-6f)
    {
        return false;
    }

    float invDet = 1.0f / det;

    result[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * invDet;
    result[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) * invDet;
    result[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * invDet;

    result[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) * invDet;
    result[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * invDet;
    result[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) * invDet;

    result[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * invDet;
    result[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) * invDet;
    result[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * invDet;

    return true;
}