#include <iostream>
#include <malloc.h>
#include <stdlib.h>
#include <math.h>
using namespace std;

double *zuigan(double *a,double *b,double *c,double *f,int n) //追赶法求线性方程组
{
double *x=NULL;
double *p=NULL;
double *q=NULL;

    x=(double*)malloc(sizeof(double)*n);
    p=(double*)malloc(sizeof(double)*(n-1));
    q=(double*)malloc(sizeof(double)*n);
double t=0;

    p[0]=f[0]/b[0];
    q[0]=c[0]/b[0];

for (int i=1;i<n;i++)
    {
        t=b[i]-a[i-1]*q[i-1];
        p[i]=(f[i]-a[i-1]*p[i-1])/t;
        q[i]=c[i]/t;
    }

for (int i=0;i<n;i++)
    {
        x[i]=p[i];
    }

for (int i=n-2;i>=0;i--)
    {
        x[i]=p[i]-q[i]*x[i+1];
    }

return x;
}


int main()
{
double x[]={-3,-2,1,3};
double y[]={2,0,3,1};
double S1=-1,S2=1;   //上下边界导数值
    double resX=1.5;
double resY=0.0;

double *h=NULL;
double *u=NULL;
double *v=NULL;    
double *b=NULL;
double *d=NULL;
int n=0;

double *m=NULL;

    n=sizeof(x)/sizeof(double);
    h=(double*)malloc(sizeof(double)*(n-1));
for (int i=0;i<n-1;i++)
    {
        h[i]=x[i+1]-x[i];
    }

    u=(double*)malloc(sizeof(double)*(n-2));
    v=(double*)malloc(sizeof(double)*(n-2));
for (int i=0;i<n-2;i++)
    {
        u[i]=h[i]/(h[i]+h[i+1]);
        v[i]=1-u[i];
    }
    u[n-2]=1;

    d=(double*)malloc(sizeof(double)*n);
for (int i=1;i<n-1;i++)
    {
        d[i]=(6/(h[i-1]+h[i]))*((y[i+1]-y[i])/h[i]-(y[i]-y[i-1])/h[i-1]);
    }

    d[0]=6/h[0]*((y[1]-y[0])/h[0]-S1);
    d[n-1]=6/h[n-2]*(S2-(y[n-1]-y[n-2])/h[n-2]);

    b=(double*)malloc(sizeof(double)*n);
for (int i=0;i<n;i++)
    {
        b[i]=2;
    }


double tmp=0;
for (int i=n-2;i>0;i--)
    {
        v[i]=v[i-1];

    }
    v[0]=1;

    m=(double*)malloc(sizeof(double)*n);
    m=zuigan(u,b,v,d,n);

int j=0;

for (double k=x[0];k<=x[n-1];k+=0.1)   //求拟合值，步进为0.1
    {
        resX=k;
for (int i=0;i<n;i++)
        {
if (resX>=x[i]&&resX<=x[i+1])
            {
                j=i;
break;
            }
        }    
double r1=x[j+1]-resX;
double r2=resX-x[j];

        resY=(pow(r1,3.0)/(6.0*h[j]))*m[j]+
               (pow(r2,3.0)/(6.0*h[j]))*m[j+1]+
             (y[j]-m[j]*h[j]*h[j]/6.0)*(r1/h[j])+
             (y[j+1]-m[j+1]*h[j]*h[j]/6.0)*(r2/h[j]);

        cout<<resY<<endl;
    }

    system("pause");
return 0;
}
