#include <iostream>
using namespace std;

// 牛顿迭代法
// f(x) = X^3 - y, 当f(x) =0时,求x
// 根据牛顿迭代法。Xn+1 = Xn - f(Xn)/ f'(Xn)
// 所以 xn+1 =  x- (X^3 -y)/(3X^2) = X*2/3 + y/(*X^2) * 1/3
// Xn+1 = (X)/3
inline double abs(double x){return (x>0?x:-x);}
double getcudeRoot(double y) {
   double x =1.0;
   for (x = 1.0; abs(x*x*x -y) > 1e-6; x=(2*x+y/x/x)/3);
   return x;
}


int main() {
    double input;
    while (cin >> input) {
        printf("%.1f\n", getcudeRoot(input));
    }
    return 0;
}
