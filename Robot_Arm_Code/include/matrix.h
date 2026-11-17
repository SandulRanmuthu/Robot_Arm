#ifndef MATRIX_H
#define MATRIX_H

// Matrix multiplication: 3x3 * 3x3
void matrixMultiply(
    const float A[3][3],
    const float B[3][3],
    float result[3][3]
);

// Matrix-vector multiplication: 3x3 * 3x1
void matrixVectorMultiply(
    const float A[3][3],
    const float B[3],
    float result[3]
);

// Calculate determinant of 3x3 matrix
float matrixDeterminant(
    const float A[3][3]
);

// Calculate inverse of 3x3 matrix
bool matrixInverse(
    const float A[3][3],
    float result[3][3]
);

#endif