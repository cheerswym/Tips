#include <iostream>
#include <vector>
#include <tuple>
int main(){
std::vector<std::tuple<double, double, double>> workspaceBoundary; 


for (const auto &bound : workspaceBoundary) {
    if (std::get<0>(bound) > 10.0) {

    }
}
}
