#include <stdbool.h>

#include "helpermath.h"

void initialize(float*** matrix, int height, int width);

void identity(float** matrix, int dim);

void addMatrices(float** matrix1, float** matrix2, int height, int width, bool positive);

bool dotProduct(float** resultant, float** matrix1, int height1, int width1, float** matrix2, int height2, int width2);

void transpose(float** resultant, float** matrix, int height, int width);

void printMatrix(float** matrix, int height, int width);

void inverse2X2(float** matrix, float** resultant);

void zero(float** matrix, int height, int width);

void copy(float** matrix1, float** matrix2, int height, int width);