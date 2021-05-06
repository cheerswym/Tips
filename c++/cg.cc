#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<string.h>
#define N 1
#define epsilon  0.005


/*
	矩阵A和向量b，相乘结果存在Ab里
*/
void matrixTimesVec(double A[N][N], double b[N], double Ab[N])
{
	int i, j;


	for (i = 0; i < N; i++)
	{
		Ab[i] = 0.0;
		for (j = 0; j < N; j++)
		{
			Ab[i] = Ab[i] + A[i][j] * b[j];
		}
	}
}
/*
	返回两个向量的内积
*/
double scalarProduct(double vec1[], double vec2[])
{
	double s = 0;
	int i;
	for (i = 0; i < N; i++)
	{
		s = s + vec1[i] * vec2[i];
	}
	return s;
}
/*
	向量vec1，vec2之和存在vec里
*/
void vecPlus(double vec1[], double vec2[], double vec[])
{
	int i;
	for (i = 0; i < N; i++)
	{
		vec[i] = vec1[i] + vec2[i];
	}
}
/*
	vec0向量每个元素乘以系数num，保存在vec中
*/
void numPlusVec(double num, double vec0[], double vec[])
{
	int i;
	for (i = 0; i < N; i++)
		vec[i] = num * vec0[i];

}

int main()
{
	//原问题：Ax=b
	//求解二次泛函：fi(x)=1/2x^TAx-b^Tx
	int i, j;
	/*
		初始化
	*/
	static double A[N][N] = { 1};//A(要求对称 + 正定)
	static double b[N] = { 0};//b
	static double x0[N] = { 1.4};//初始解x0
	double x[N], r[N], p[N], w[N], alpha, rho00, rho0, rho1, beta;
	//打印
	printf("\n要求解的示例方程组为：\n A ||| b \n");
	for (i = 0; i < N; i++)
	{
		for (j = 0; j < N; j++)
		{
			printf("%f ", A[i][j]);
		}
		printf("||| %f\n", b[i]);
	}
	printf("初始解x0为：\n");
	for (int i = 0; i < N; i++) {
		printf("%f ",x0[i]);
	}
	//init
	memcpy(x, x0, N * sizeof(double));//x初始化为x0
	double Ax0[N];
	matrixTimesVec(A, x0, Ax0);//计算矩阵A和向量x0的乘积，存在Ax0
	numPlusVec(-1.0, Ax0, Ax0);//Ax0这个向量乘以-1
	vecPlus(b, Ax0, r);		   //-Ax0+b=r (计算残量)
	rho0 = scalarProduct(r, r);//计算r^Tr
	rho00 = rho0;			   //
	memcpy(p, r, N * sizeof(double));//初始化p=r（p为下降方向）
	int iter = 0;//记录迭代次数
	do
	{
		matrixTimesVec(A, p, w);//w = Ap
		alpha = rho0 / (scalarProduct(p, w));//alpha = r^Tr/p^TAp
		double temp[N];
		numPlusVec(alpha, p, temp);//p*alpha,存在temp里
		double x_temp[N];
		vecPlus(x, temp, x_temp);//更新x (对应伪代码第10行)
		memcpy(x, x_temp, N * sizeof(double));//同上

		numPlusVec(-alpha, w, temp);//-alpha*A*p,存在temp里
		double r_temp[N];
		vecPlus(r, temp, r_temp);//更新r(伪代码第11行)
		memcpy(r, r_temp, N * sizeof(double));//同上
		//下面两行对应伪代码12行
		rho1 = scalarProduct(r, r);//r(k+1)^Tr(k+1)
		beta = rho1 / rho0;//beta=r(k+1)^Tr(k+1)/r(k)^Tr(k)
		//下面两行对应伪代码13行
		numPlusVec(beta, p, temp);//b*p,存在temp
		vecPlus(r, temp, p);//更新方向向量p
		rho0 = rho1;
		iter++;
	} while (rho1 > epsilon);//看看残差r^Tr是否小于阈值了

	printf("\n\n");
	printf("方程组的解为：\n");
	for (i = 0; i < N; i++)
		printf("%f\n", x[i]);
	printf("迭代了%d次", iter);
	//getchar();
	return 0;
}
