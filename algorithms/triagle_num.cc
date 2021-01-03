#include <algorithm>
#include <iostream>
using namespace std;

int D[20][20];
int n;
int maxsum[20][20];
/*递归法
int MaxSum(int i,int j)
{
    //表示已经计算过这个值了，可以直接取 不用重复计算
    if( maxsum[i][j] != -1 )
                return maxsum[i][j];
                //最后一行
    if(i == n-1)
        maxsum[i][j] = D[n-1][j];
    else{
        maxsum[i][j] =max(MaxSum(i+1,j+1),MaxSum(i+1,j))+D[i][j];
    }
    return maxsum[i][j];
}
*/

int main() {
  cin >> n;
  for (int i = 0; i < n; i++)
    for (int j = 0; j <= i; j++) {
      cin >> D[i][j];
      maxsum[i][j] = -1;
    }
  //递推法，从下往上计算
  for (int j = 0; j < n; j++) maxsum[n - 1][j] = D[n - 1][j];
  for (int i = n - 2; i >= 0; i--)
    for (int j = 0; j <= i; j++)
      maxsum[i][j] = max(maxsum[i + 1][j], maxsum[i + 1][j + 1]) + D[i][j];
  cout << maxsum[0][0];

  return 0;
}
