#include <iostream>
#include <cstring>
#include <cstdio>
using namespace std;
const int maxn=21+4;
int map[maxn][maxn]={0};//用来存储地图
int mark[maxn];//用来记录走过的点，以便输出
bool m[maxn];//用来记忆该点有没有被访问过
int n,ans1;//ans1记录总共有多少条路径
void dfs(int x,int ans)//x表示所在的地点，ans表示这是去到的第几个地点
{
    if(x==n)
    {
        ans1++;
        cout<<'1';
        for(int i=1;i<ans;i++)
            cout<<' '<<mark[i];
        cout<<endl;return ;
    }
    for(int i=2;i<maxn;i++)//从2开始逐个搜索，因为是从1出发，不能再回到1
        if(map[x][i]==1&&!m[i])//如果i点与x点连通，而且i点没被访问过
        {
            mark[ans]=i;m[i]=true;
            dfs(i,ans+1);
            m[i]=false;
        }
    return ;
}
int main()
{
    //freopen("text.txt","r",stdin);//文件输入
    int cnt=1;
    while(cin>>n)
    {
        memset(map,0,sizeof(map));//一轮下来，有些数据要被重置
        int x=1,y=1;
        ans1=0;//重置
        while(true)
        {
            cin>>x>>y;
            if(x==0||y==0)break;
            map[x][y]=map[y][x]=1;//构建矩阵地图
        }
        cout<<"CASE "<<cnt++<<":"<<endl;
        dfs(1,1);
        cout<<"There are "<<ans1<<" routes from the firestation to streetcorner "<<n<<"."<<endl;
    }
    return 0;
}
