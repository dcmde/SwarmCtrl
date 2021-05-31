#include <iostream>
#include <SwarmCtrl.hpp>

int main() {
    std::vector<std::vector<double>> vec(3);
    vec[0] = std::vector<double>({0, 0});
    vec[1] = std::vector<double>({5, 0});
    vec[2] = std::vector<double>({6, 0});

    std::vector<double> uPos({0, 8});

    SwarmCtrl swarmCtrl;
    swarmCtrl.setRepParam(1);
    swarmCtrl.setAttractParam(1);
    swarmCtrl.setMaxIteration(10);
    swarmCtrl.setSampleTime(0.02);
    swarmCtrl.getOptimalPosition(vec, uPos);

    printVec(swarmCtrl.getOptimalPosition(vec, uPos));
    return 0;
}
