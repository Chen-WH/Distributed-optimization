#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <math.h>
#include <iomanip>
#include <time.h>
#include <fstream>
#include <map>
#include "mkl.h"
#include "udf.h"

using namespace std;

class Matrix
{
private:
	int row, col;
	double* index;
public:
	Matrix(int x = 1, int y = 1) {
		row = x;
		col = y;
		index = (double *)mkl_malloc( row*col*sizeof( double ), 64 );
	}

	Matrix(const Matrix& obj) {
		row = obj.row;
		col = obj.col;
		index = (double *)mkl_malloc( row*col*sizeof( double ), 64 );
		cblas_dcopy(row*col, obj.index, 1, index, 1);
	}

	Matrix(double **array, int x, int y) {
		row = x;
		col = y;
		index = (double *)mkl_malloc( row*col*sizeof( double ), 64 );
		for (int i = 0; i < row; ++i) {
			cblas_dcopy(row*col, array[i], 1, index + i*col, 1);
		}
	}

	~Matrix() {
		mkl_free(index);
	}

	void E() {
		for (int i = 0; i < (row*col); ++i) {
			index[i] = 0;
		}
		for (int i = 0; i < row; ++i) {
			index[i*col+i] = 1;
		}
	}

	void zero() {
		for (int i = 0; i < (row*col); ++i) {
			index[i] = 0;
		}
	}

	friend istream& operator>>(istream& is, Matrix& obj) {
		for (int i = 0; i < (obj.row*obj.col); ++i) {
			is >> obj.index[i];
		}
		return is;
	}

	friend ostream& operator<<(ostream& os, const Matrix& obj) {
		if (obj.row == 0 || obj.col == 0) {
			os << "ERROR!";          //不符合矩阵运算时自动报错
		}
		else {
			for (int i = 0; i < obj.row; ++i) {
				for (int j = 0; j < obj.col; ++j) {
					os << obj.index[i*obj.col + j] << ' ';
				}
				os << endl;
			}
		}
		return os;
	}

	int Row() {
		return row;
	}

	int Col() {
		return col;
	}

	double& operator()(int x, int y) {
		return index[x*col + y];
	}

	// *this = *this + obj
	Matrix add(const Matrix obj) {
		if (obj.row != row || obj.col != col) {
			printf("相加矩阵维数不同!\n");
			return *this;
		}
		else {
			cblas_daxpy(row*col, 1, obj.index, 1, index, 1);
			return *this;
		}
	}

	// *this = *this - obj
	Matrix minus(const Matrix& obj) {
		if (obj.row != row || obj.col != col) {
			printf("相加矩阵维数不同!\n");
			return *this;
		}
		else {
			cblas_daxpy(row*col, -1, obj.index, 1, index, 1);
			return *this;
		}
	}

	// *this = *this * obj
	Matrix multp(const Matrix& obj) {
		if (col != obj.row) {
			cout << "维数不符合矩阵相乘原则！" << endl;
		}
		else {
			cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, row, obj.col, col, 1, index, col, obj.index, obj.col, 0, index, obj.col);
			return *this;
		}
	}

	Matrix multp(const double x) {
		cblas_dscal(row*col, x, index, 1);
		return *this;
	}

	Matrix div(const double x) {
		cblas_dscal(row*col, 1/x, index, 1);
		return *this;
	}

	Matrix operator+(const Matrix& obj) {
		Matrix tmp(row, col);
		if (obj.row != row || obj.col != col) {
			printf("相加矩阵维数不同!\n");
			return *this;
		}
		else {
			tmp.zero();
			cblas_daxpy(row*col, 1, obj.index, 1, tmp.index, 1);
			cblas_daxpy(row*col, 1, index, 1, tmp.index, 1);
			return tmp;
		}
	}

	// *this = *this - obj
	Matrix operator-(const Matrix& obj) {
		Matrix tmp(row, col);
		if (obj.row != row || obj.col != col) {
			printf("相加矩阵维数不同!\n");
			return *this;
		}
		else {
			tmp.zero();
			cblas_daxpy(row*col, -1, obj.index, 1, tmp.index, 1);
			cblas_daxpy(row*col, 1, index, 1, tmp.index, 1);
			return tmp;
		}
	}

	friend Matrix operator*(const Matrix& obj1, const Matrix& obj2) {
		if (obj1.col != obj2.row) {
			cout << "维数不符合矩阵相乘原则！" << endl;
			Matrix tmp;
			return tmp;
		}
		else {
			Matrix tmp(obj1.row, obj2.col);
			cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, obj1.row, obj2.col, obj1.col, 1, obj1.index, obj1.col, obj2.index, obj2.col, 0, tmp.index, obj2.col);
			return tmp;
		}
	}

	friend Matrix operator*(const Matrix& obj, const double x) {
		Matrix tmp(obj);
		cblas_dscal(obj.row*obj.col, x, tmp.index, 1);
		return tmp;
	}

	friend Matrix operator/(const Matrix& obj, const double x) {
		Matrix tmp(obj);
		cblas_dscal(obj.row*obj.col, 1/x, tmp.index, 1);
		return tmp;
	}

	Matrix& operator=(const Matrix& obj) {
		mkl_free(index);
		row = obj.row;
		col = obj.col;
		index = (double *)mkl_malloc( row*col*sizeof( double ), 64 );
		cblas_dcopy(row*col, obj.index, 1, index, 1);
		return *this;
	}

	//矩阵转置
	Matrix trans() {
		Matrix tmp(col, row);
		for (int i = 0; i < col; ++i) {
			cblas_dcopy(row, index + i, col, tmp.index + i*row, 1);
		}
		return tmp;
	}

	//分块矩阵
	Matrix block(int x, int y, int new_row, int new_col) {
		Matrix tmp(new_row, new_col);
		for (int i = 0; i < new_row; ++i) {
			cblas_dcopy(new_col, index + (i + x)*col, 1, tmp.index + i*new_col, 1);
		}
		return tmp;
	}

	//行交换
	void swaprow(const int i, const int j) {
		cblas_dswap(col, index + i*col, 1, index + j*col, 1);
	}

	//行变换
	void rowmultik(int k, double x) {
		cblas_dscal(col, x, index + k*col, 1);
	}

	//求解线性方程组
	Matrix LUsolve(const Matrix b, bool& flag) {
		Matrix A(*this);
		Matrix x(b);
		int info;
		int inc = 1;
		char trans = 'N';
		int* ipiv = (int *)mkl_malloc( minInt(row, col)*sizeof( int ), 64 );
		dgetrf(&row, &col, A.index, &row, ipiv, &info);
		if (info < 0){
			flag = false;
			return b;
		}
		dgetrs(&trans, &row, &inc, A.index, &row, ipiv, x.index, &row, &info);
		return x;
	}

	//求解线性方程组(对称)
	/*
	Matrix solve(const Matrix b, bool& flag) {
		Matrix A(*this);
		Matrix x(b);
		int info;
		int inc = 1;
		int* ipiv = (int *)mkl_malloc( row*sizeof( int ), 64 );
		dgesv(&row, &inc, A.index, &row, ipiv, x.index, &row, &info);
		return x;
	}*/

	Matrix solve(const Matrix b, bool& flag) {
		Matrix A(*this);
		Matrix x(b);
		int info;
		int inc = 1;
		int* ipiv = (int *)mkl_malloc( row*sizeof( int ), 64 );
		mkl_sparse_dtrsv(&row, &inc, A.index, &row, ipiv, x.index, &row, &info);
		return x;
	}

	//一维范数
	double norm() {
		return cblas_dnrm2(row*col, index, 1);
	}
	
	//求取两向量夹角
	friend double theta(Matrix obj1, Matrix obj2)
	{
		if (obj1.row != obj2.row || obj1.col != 1 || obj2.col != 1) {
			cout << "向量维数不相等！" << endl;
			return 0;
		}
		return cblas_ddot(obj1.row, obj1.index, 1, obj2.index, 1) / obj1.norm() / obj2.norm();
	}
};
