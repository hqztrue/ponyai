#ifndef _MATRIX_H_
#define _MATRIX_H_
#include "common.h"
/*
#define MAX 10

void Inverse(double *matrix1[],double *matrix2[],int n,double d);
double Determinant(double* matrix[],int n);
double AlCo(double* matrix[],int jie,int row,int column);
double Cofactor(double* matrix[],int jie,int row,int column);

void Inverse(double *matrix1[],double *matrix2[],int n,double d) 
{ 
    int i,j; 
    for(i=0;i<n;i++) 
    {
		matrix2[i]=(double *)malloc(n*sizeof(double)); 
	}
    for(i=0;i<n;i++) 
    {
		for(j=0;j<n;j++) 
        {
			*(matrix2[j]+i)=(AlCo(matrix1,n,i,j)/d); 
		}
	}
} 

double Determinant(double* matrix[],int n)  
{  
    double result=0,temp;  
    int i;  
    if(n==1)  
    {
		result=(*matrix[0]);  
	}
    else  
    {  
        for(i=0;i<n;i++)  
        {  
            temp=AlCo(matrix,n,n-1,i);  
            result+=(*(matrix[n-1]+i))*temp;  
        }  
    }  
    return result;  
}  

double AlCo(double* matrix[],int jie,int row,int column)  
{  
    double result; 
    if((row+column)%2 == 0) 
    {
		result = Cofactor(matrix,jie,row,column);  
	}
    else 
	{
		result=(-1)*Cofactor(matrix,jie,row,column); 
	}

    return result;  
}  

double Cofactor(double* matrix[],int jie,int row,int column)  
{  
    double result;  
    int i,j;  
    double* smallmatr[MAX-1];  
    for(i=0;i<jie-1;i++)  
    {
		smallmatr[i]= new double[jie - 1];
	}
    for(i=0;i<row;i++)  
    {    
		for(j=0;j<column;j++)  
        {
			*(smallmatr[i]+j)=*(matrix[i]+j);  
		}
	}
    for(i=row;i<jie-1;i++)  
    {
		for(j=0;j<column;j++)  
        {
			*(smallmatr[i]+j)=*(matrix[i+1]+j);  
		}
	}
    for(i=0;i<row;i++)  
    {
		for(j=column;j<jie-1;j++)  
        {
			*(smallmatr[i]+j)=*(matrix[i]+j+1);  
		}
	}
    for(i=row;i<jie-1;i++)  
    {
		for(j=column;j<jie-1;j++)  
        {
			*(smallmatr[i]+j)=*(matrix[i+1]+j+1);  
		}
	}
    result = Determinant(smallmatr,jie-1); 
    for(i=0;i<jie-1;i++)
    {
		delete[] smallmatr[i];
	}

    return result;   
}*/

namespace Matrix{	template<size_t Size>	struct matrix{		double key[Size][Size];		void clear(){ memset(key, 0, sizeof(key)); }		matrix(){ clear(); }		void read(){			for (int i = 0; i<Size; ++i)				for (int j = 0; j<Size; ++j)					scanf("%lf", &key[i][j]);		}		void print()const{			for (int i = 0; i<Size; ++i)				for (int j = 0; j<Size; ++j)					printf("%.9lf%c", key[i][j], j + 1 == Size ? '\n' : ' ');		}		matrix<Size> inverse()const{			matrix A(*this);			matrix<Size> ans;			for (int i = 0; i<Size; ++i)				ans.key[i][i] = 1;			for (int i = 0; i<Size; ++i){				int id = -1;				for (int j = i; j<Size; ++j)					if (fabs(A.key[j][i])>eps){						id = j; break;					}				if (id == -1){					ans.clear();					return ans;				}				if (id>i){					double tmp;					for (int k = i; k<Size; ++k)						tmp = A.key[i][k], A.key[i][k] = A.key[id][k], A.key[id][k] = tmp;					for (int k = 0; k<Size; ++k)						tmp = ans.key[i][k], ans.key[i][k] = ans.key[id][k], ans.key[id][k] = tmp;				}				double d = A.key[i][i];				for (int k = i; k<Size; ++k)					A.key[i][k] /= d;				for (int k = 0; k<Size; ++k)					ans.key[i][k] /= d;				for (int j = 0; j<Size; ++j)					if (j != i && fabs(A.key[j][i])>eps){						double t = A.key[j][i];						for (int k = i; k<Size; ++k)							A.key[j][k] -= A.key[i][k] * t;						for (int k = 0; k<Size; ++k)							ans.key[j][k] -= ans.key[i][k] * t;					}			}			return ans;		}		matrix<Size> transpose()const{			matrix<Size> ans;			for (int i = 0; i<Size; ++i)				for (int j = 0; j<Size; ++j)					ans[i][j] = key[j][i];			return ans;		}		double det(){			static double key_[Size][Size];			for (int i = 0; i<Size; ++i)				for (int j = 0; j<Size; ++j)					key_[i][j] = key[i][j];			double ans = 1;			for (int i = 0; i<Size; ++i){				int id = -1;				for (int j = i; j<Size; ++j)					if (fabs(key_[j][i])>eps){						id = j; break;					}				if (id == -1)return 0;				if (id>i){					double tmp;					for (int k = i; k<Size; ++k)						tmp = key_[i][k], key_[i][k] = key_[id][k], key_[id][k] = tmp;					ans *= -1;				}				for (int j = i + 1; j<Size; ++j)					if (fabs(key_[j][i])>eps){						double t = key_[j][i] / key_[i][i];						for (int k = i; k<Size; ++k)							key_[j][k] -= key_[i][k] * t;					}				ans *= key_[i][i];			}			return ans;		}		double* operator [](int x){ return key[x]; }		const double* operator [](int x)const{ return key[x]; }	};	template<size_t Size>	const matrix<Size>& operator +(const matrix<Size> &a, const matrix<Size> &b){		static matrix<Size> T; T.clear();		for (int i = 0; i<Size; ++i)			for (int j = 0; j<Size; ++j)T.key[i][j] = a.key[i][j] + b.key[i][j];		return T;	}	template<size_t Size>	const matrix<Size>& operator -(const matrix<Size> &a, const matrix<Size> &b){		static matrix<Size> T; T.clear();		for (int i = 0; i<Size; ++i)			for (int j = 0; j<Size; ++j)T.key[i][j] = a.key[i][j] - b.key[i][j];		return T;	}	template<size_t Size>	matrix<Size> operator *(const matrix<Size> &a, const matrix<Size> &b){		matrix<Size> T;		for (int k = 0; k<Size; ++k)			for (int i = 0; i<Size; ++i)				for (int j = 0; j<Size; ++j)					T.key[i][j] += a.key[i][k] * b.key[k][j];		return T;	}	template<size_t Size>	matrix<Size> pow(const matrix<Size> &x, int y){		static matrix<Size> res, tmp; tmp = x;		memset(res.key, 0, sizeof(res.key));		for (int i = 0; i<Size; ++i)res.key[i][i] = 1;		while (y){			if (y & 1)res = res*tmp;			tmp = tmp*tmp; y >>= 1;		}		return res;	}};using Matrix::matrix;


#endif


