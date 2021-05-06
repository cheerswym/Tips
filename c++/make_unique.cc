#include <iostream>
#include <iomanip>
#include <memory>
 
class geometry {
 public:
 geometry(){};
 int w;	
};

 
struct Vec3
{
    int x, y, z;
 
    // following constructor is no longer needed since C++20
    Vec3(int x = 0, int y = 0, int z = 0) noexcept : x(x), y(y), z(z) { }
 
    friend std::ostream& operator<<(std::ostream& os, const Vec3& v) {
        return os << "{ x=" << v.x << ", y=" << v.y << ", z=" << v.z << " }";
    }
    std::unique_ptr<geometry> geo;
};

 
int main()
{
	Vec3 vec;
	vec.geo = std::make_unique<geometry>();
	
	
    // Use the default constructor.
    std::unique_ptr<Vec3> v1 ;
    v1= std::make_unique<Vec3>();
    // Use the constructor that matches these arguments
    std::unique_ptr<Vec3> v2 = std::make_unique<Vec3>(0,1,2);
    // Create a unique_ptr to an array of 5 elements
    std::unique_ptr<Vec3[]> v3 = std::make_unique<Vec3[]>(5);
 
    std::cout << "make_unique<Vec3>():      " << *v1 << '\n'
              << "make_unique<Vec3>(0,1,2): " << *v2 << '\n'
              << "make_unique<Vec3[]>(5):   ";
    for (int i = 0; i < 5; i++) {
        std::cout << std::setw(i ? 30 : 0) << v3[i] << '\n';
    }
}
