#include <queue>
#include <stack>
#include <ctime>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <string>
#include <cstring>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <algorithm>

using namespace std;

typedef long long LL;
typedef long double LB;
typedef unsigned long long ULL;
typedef pair<int, int> PII;
typedef pair<LL, LL> PLL;
typedef vector<int> VI;

const int INF = 0x3f3f3f3f;
const LL INFL = 0x3f3f3f3f3f3f3f3fLL;
const long double PI = acos(-1.0);
const long double eps = 1e-6;
void debug() { cout << endl; }
template<typename T, typename ...R> void debug (T f, R ...r) { cout << "[" << f << "]"; debug (r...); }
template<typename T> inline void umax(T &a, T b) { a = max(a, b); }
template<typename T> inline void umin(T &a, T b) { a = min(a, b); }
template <typename T> inline bool scan_d (T &ret) {
    char c;
    int sgn;
    if (c = getchar(), c == EOF) return 0; //EOF
    while (c != '-' && (c < '0' || c > '9') ) {
        if((c = getchar()) == EOF) return 0;
    }
    sgn = (c == '-') ? -1 : 1;
    ret = (c == '-') ? 0 : (c - '0');
    while (c = getchar(), c >= '0' && c <= '9') ret = ret * 10 + (c - '0');
    ret *= sgn;
    return 1;
}
template<typename T> void print(T x) {
    static char s[33], *s1; s1 = s;
    if (!x) *s1++ = '0';
    if (x < 0) putchar('-'), x = -x;
    while(x) *s1++ = (x % 10 + '0'), x /= 10;
    while(s1-- != s) putchar(*s1);
}
template<typename T> void println(T x) {
    print(x); putchar('\n');
}
const int MAXN = 200000 + 5;

struct Node {
    LL xy[2];
    LL minx, maxx;
    LL miny, maxy;
    int f, id, ls, rs;
    int val, sum;
} A[MAXN];
int kd_cmp;
LL xl, xr, yl, yr;
inline bool cmp(const Node &a, const Node &b) { return a.xy[kd_cmp] < b.xy[kd_cmp]; }

struct KDTree {
    void push_up(int rt) {
        A[rt].minx = A[rt].maxx = A[rt].xy[0];
        A[rt].miny = A[rt].maxy = A[rt].xy[1];
        A[rt].sum = A[rt].val;
        if(A[rt].ls) {
            umin(A[rt].minx, A[A[rt].ls].minx);
            umax(A[rt].maxx, A[A[rt].ls].maxx);
            umin(A[rt].miny, A[A[rt].ls].miny);
            umax(A[rt].maxy, A[A[rt].ls].maxy);
            A[rt].sum += A[A[rt].ls].sum;
        }
        if(A[rt].rs) {
            umin(A[rt].minx, A[A[rt].rs].minx);
            umax(A[rt].maxx, A[A[rt].rs].maxx);
            umin(A[rt].miny, A[A[rt].rs].miny);
            umax(A[rt].maxy, A[A[rt].rs].maxy);
            A[rt].sum += A[A[rt].rs].sum;
        }
    }
    int build(int l, int r, int w, int fa) {
        int m = (l + r) >> 1; kd_cmp = w;
        nth_element(A + l, A + m, A + r + 1, cmp);
        A[m].val = A[m].sum = 1; A[m].f = fa;
        A[m].ls = l != m ? build(l, m - 1, !w, m) : 0;
        A[m].rs = r != m ? build(m + 1, r, !w, m) : 0;
        push_up(m);
        return m;
    }
    int query(int rt) {
        if(A[rt].minx > xr || A[rt].maxx < xl || A[rt].miny > yr || A[rt].maxy < yl)
            return 0;
        if(xl <= A[rt].minx && A[rt].maxx <= xr && yl <= A[rt].miny && A[rt].maxy <= yr)
            return A[rt].sum;
        int ret = 0;
        if(xl <= A[rt].xy[0] && A[rt].xy[0] <= xr && yl <= A[rt].xy[1] && A[rt].xy[1] <= yr)
            ret += A[rt].val;
        if(ret > 0) return ret;
        if(A[rt].ls) ret += query(A[rt].ls);
        if(ret > 0) return ret;
        if(A[rt].rs) ret += query(A[rt].rs);
        return ret;
    }
    /**
    void update(int rt, int x) {
        A[rt].val += x;
        while(rt) {
            A[rt].sum += x;
            rt = A[rt].f;
        }
    }*/
} kdtree;

int n, q;
LL qx, qy;

int main() {
#ifdef ___LOCAL_WONZY___
    freopen ("input.txt", "r", stdin);
#endif // ___LOCAL_WONZY___
    while(scan_d(n)) {
        if(n == -1) break;
        for(int i = 1; i <= n; i++) {
            scan_d(qx); scan_d(qy);
            A[i].xy[0] = qx - qy;
            A[i].xy[1] = qx + qy;
        }
        int root = kdtree.build(1, n, 0, 0);
        scan_d(q);
        while(q --) {
            scan_d(qx); scan_d(qy);
            LL lb = 0, ub = 2e9, mid;
            while(lb <= ub) {
                mid = (lb + ub) >> 1;
                xl = (qx - mid) - qy;
                yl = (qx - mid) + qy;
                xr = (qx + mid) - qy;
                yr = (qx + mid) + qy;
                if(kdtree.query(root) >= 1) ub = mid - 1;
                else lb = mid + 1;
            }
            println(lb);
        }
        puts("");
    }
#ifdef ___LOCAL_WONZY___
    cout << "Time elapsed: " << 1.0 * clock() / CLOCKS_PER_SEC * 1000 << " ms." << endl;
#endif // ___LOCAL_WONZY___
    return 0;
}
