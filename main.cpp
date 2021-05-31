#include <SwarmCtrl.hpp>

int main() {
    std::vector<std::vector<double>> vec(1);
    vec[0] = std::vector<double>({-1, 0});
//    vec[1] = std::vector<double>({1, 0});
//    vec[2] = std::vector<double>({-1, 0});

    std::vector<double> uPos({1, 0});

    SwarmCtrl swarmCtrl;
    swarmCtrl.setRepParam(10);
    swarmCtrl.setAttractParam(1);
    swarmCtrl.setMaxIteration(900);
    swarmCtrl.setSampleTime(0.05);
    swarmCtrl.setBorders(std::vector<double>({2, 0, 2, 0}));
    auto it = swarmCtrl.getOptimalPosition(vec, uPos);
    printVec(it);
    return 0;
}
