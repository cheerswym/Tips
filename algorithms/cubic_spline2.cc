#include "stdio.h"
#include<iostream>
#include<iomanip>
#define N 50
int main(int argc, char* argv[])
{
	int i,j,k,n;
	double X[N],Y[N],h[N],u[N],d[N],m[N],a[N],b[N],A[N][N],S[N][4];
	void line_equations(double A[][50],double X[],int n=1);
//变量的定义    说明: k=0 to n-1  u[0] =m[k]*a[k];   
//	                  k=0 to n-1  u[n-1] =m[k]*b[k];    为附加的两个条件
	cout<<"请输入点的个数n:";
	cin>>n;
	cout<<"请输入各个点的坐标:"<<endl;
	for(i=0;i<=n-1;i  ){
		cout<<"X["<<i<<"]=";
			cin>>X[i];
		cout<<"Y["<<i<<"]=";
			cin>>Y[i];
	};
//坐标的输入
	cout<<"请输入两个附加条件:m[k]的线性组合"<<endl;
	for(i=0;i<=n-1;i  ){
		cout<<" m["<<i<<"]*";
		cin>>a[i];
	};
	cout<<"=";
	cin>>u[0];
	for(i=0;i<=n-1;i  ){
		cout<<" m["<<i<<"]*";
		cin>>b[i];
	};
	cout<<"=";
	cin>>u[n-1];
//附加条件的输入
	for(k=0;k<=n-2;k  ){
		h[k]=X[k 1]-X[k];
		d[k]=(Y[k 1]-Y[k])/(X[k 1]-X[k]);
	};
	if(n>=3){
		for(k=1;k<=n-2;k  ){
		    u[k]=6*(d[k]-d[k-1]);
		};
		for(i=1;i<=n-2;i  ){
	    	for(j=0;j<=n;j  ){
			A[i][j]=0;
			}
		};
	};
//初始化方程组的系数增广矩阵
    for(i=0;i<=n-1;i  ){
	    	A[0][i]=a[i];
		    A[n-1][i]=b[i];
		    A[i][n]=u[i];
	};
	if(n>=3){
		for(i=1;i<=n-2;i  ){
			A[i][i-1]=h[i-1];
			A[i][i]=2*(h[i-1] h[i]);
			A[i][i 1]=h[i];
		}
	};
//方程组的系数增广矩阵的填充	
	line_equations(A,m,n);
//求解m[k];
	for(k=0;k<=n-2;k  ){
		S[k][0]=m[k]*X[k 1]*X[k 1]*X[k 1]/(6*h[k]) (Y[k]/h[k]-m[k]*h[k]/6)*X[k 1];
		S[k][0]-=m[k 1]/(6*h[k])*X[k]*X[k]*X[k] (Y[k 1]/h[k]-m[k 1]*h[k]/6)*X[k];
		S[k][1]=m[k 1]*X[k]*X[k]/(2*h[k]) (Y[k 1]/h[k]-m[k 1]*h[k]/6);
		S[k][1]-=m[k]*X[k 1]*X[k 1]/(2*h[k]) (Y[k]/h[k]-m[k]*h[k]/6);
		S[k][2]=m[k]*X[k 1]/(2*h[k])-m[k 1]*X[k]/(2*h[k]);
		S[k][3]=(m[k 1]-m[k])/(6*h[k]);
	};
//求解三次样条曲线的各次方的系数
	cout<<"下面是三次样条曲线:"<<endl;
	for(i=0;i<=n-2;i  ){
		cout<<"S["<<i<<"]["<<"X]= "<<S[i][3]<<" *X^3 ";
		if(S[i][2]>=0) cout<<" ";
		cout<<S[i][2]<<" *X^2 ";
		if(S[i][1]>=0) cout<<" ";
		cout<<S[i][1]<<" *X ";
		if(S[i][0]>=0) cout<<" ";
		cout<<S[i][0]<<endl;
	};
//输出三次样条曲线多项式
	cin.get();
	cin.get();
	return 0;
};

void line_equations(double A[][50],double X[],int n=1){
	int i,j,k,s=0;
	double temp;
	if(n==1 && A[0][0]!=0){
		X[0]=A[0][1]/A[0][0];
		return ;
	};
//为一阶时直接求出解
	for(k=0;k<=n-2;k  ){
		for(i=k;i<=n;i  ){
			if(A[i][k]!=0) break;
		};                         //求出从上往下第一个不为0的行
		if(i>=n){
			cout<<"无解!"; return ;
		}; //若在化归上三角矩阵的时候碰到有缺阶数的时候，此时无解或有无穷多解，统一为无解
		if(i!=k){
			for(j=k;j<=n;j  ){
				temp=A[k][j];
				A[k][j]=A[i][j];
				A[i][j]=temp;
			};
		};                        //第一个不为0的行来与k行交换，以免除数为0
		for(i=k 1;i<=n-1;i  ){
			for(j=k 1;j<=n;j  ){
				A[i][j]-=((A[i][k]/A[k][k])*A[k][j]);
			};
			A[i][k]=0;
		};                 //第k次循环时，将第1列化为0
	};
//至此，将方程组的系数增广矩阵化为了上三角阵
	X[n-1]=A[n-1][n]/A[n-1][n-1];   //从下向上迭代求出X[k],先求最后一个
	for(i=n-2;i>=0;i--){
		for(j=n-2;j>=i;j--){
			A[i][n]-=X[j 1]*A[i][j 1];
		};
		X[i]=A[i][n]/A[i][i];
	};
//迭代求出X[k]
return;
}
