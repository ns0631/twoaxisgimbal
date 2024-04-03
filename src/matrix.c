#include "matrix.h"

void initialize(float*** matrix, int height, int width)
{
   *matrix = (float **)calloc(height, sizeof(float *));
   for(int i = 0; i < height; i++) {
        (*matrix)[i] = (float *)calloc(width, sizeof(float));
   }
}

void identity(float** matrix, int dim){
	for(int i = 0; i < dim; i++){
		matrix[i][i] = 1;
	}
}

void addMatrices(float** matrix1, float** matrix2, int height, int width, bool positive){
	int factor;
	if(positive){
		factor = 1;
	} else{
		factor = -1;
	}

	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			matrix1[i][j] += factor * matrix2[i][j];
		}
	}
}

bool dotProduct(float** resultant, float** matrix1, int height1, int width1, float** matrix2, int height2, int width2){
	if(width1 != height2){
		return false;
	}

	for(int i = 0; i < height1; i++){
		for(int j = 0; j < width2; j++){
			for(int k = 0; k < width1; k++){
				resultant[i][j] += matrix1[i][k] * matrix2[k][j];
			}
		}
	}

	return true;
}

void transpose(float** resultant, float** matrix, int height, int width){
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			resultant[j][i] = matrix[i][j];
		}
	}
}


void printMatrix(float** matrix, int height, int width){
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			printk("%f ", matrix[i][j]);
		}
		printk("\n");
	}
}

void inverse2X2(float** matrix, float** resultant){
	float a = matrix[0][0];
	float b = matrix[0][1];
	float c = matrix[1][0];
	float d = matrix[1][1];

	float factor = 1 / ( a * d - b * c );
	resultant[0][0] = d * factor;
	resultant[0][1] = -b * factor;
	resultant[1][0] = -c * factor;
	resultant[1][1] = a * factor;
}

void zero(float** matrix, int height, int width){
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			matrix[i][j] = 0;
		}
	}
}

void copy(float** matrix1, float** matrix2, int height, int width){
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			matrix1[i][j] = matrix2[i][j];
		}
	}
}