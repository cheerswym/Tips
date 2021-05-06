#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <map>
#include <set>
#include <unordered_set>

#define F(x) (x*x)   //目标函数
#define dF(x) (2*x)      //函数求导
#define learning_rate 0.5   //步长
#define update(x) (x-learning_rate*dF(x)) //x更新规则

int main() {
	double x0 = -50;
	for (int i = 0; i < 1; i++)
	{
		x0 += -learning_rate*dF(x0);
	}
	printf("x=%f,y=%f\n", x0, F(x0));
	
	
	std::vector<double> traj_points;
	for (int i = 1; i < 30; ++i) {
	  traj_points.push_back(static_cast<double>(i));
	}
	traj_points[10] = 16.0;
	traj_points[4] = 1.0;
	traj_points[19] = 17.0;
	for (unsigned int i = 0; i < traj_points.size(); ++i) {
	  std::cout << traj_points[i] << "\n";
	}

	for (int loop = 0; loop < 3663443; ++loop) {
	  for (unsigned int i = 0; i < traj_points.size() - 4; ++i) {
		double v0 = traj_points[i];
		double v1 = traj_points[i + 1];
		double v2 = traj_points[i + 2];
		double v3 = traj_points[i + 3];
		double v4 = traj_points[i + 4];
		//v0 += -0.01 * (2.0 * v0 + 2.0 * v2 - 4.0 * v1);
		//v1 += -0.01 * (4 * v1 - 2.0 * v2 - 2.0 * v0);
		//traj_points[i+1] = v1;
		//v2 += -0.01 * 2.0 * (v2 + v0 - 2.0 * v1);
		//v2 += -0.01 * 2.0 * (v0 - 3.0 * v1 + 3.0 * v2 - v3); // before 2 after 1

		//v2 += -0.01 * (2.0*2.0 * (v0 + v2 - 2.0*v1) - 2.0 *(v1 + v3 - 2.0* v2));
		//v2 += -0.01 * 2.0* (v0 - 3.0*v1 +4.0*v2 - 3.0*v3 + v4);
		
		// gradient descent
		//double Q = 4.0;
		//v1 += -(1/Q) * (2.0*(v1-v0)-2.0*(v2-v1));
		//traj_points[i+1] = v1;
		
		
		// newton
		//double fx = (v1-v0)*(v1-v0) + (v2-v1)*(v2-v1);
		double gx = 2.0*(v1-v0)-2.0*(v2-v1);
		double ggx = 4;
		v1 = v1 - gx/ggx;
		//v1 = (v0+v2) /2;
		traj_points[i+1] = v1;
	  }
	}
	for (unsigned int i = 0; i < traj_points.size(); ++i) {
	  std::cout << traj_points[i] << "\n";
	}
	

}
