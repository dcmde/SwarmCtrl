

#include <SwarmCtrl.hpp>

void SwarmCtrl::setBorders(const std::vector<double> &borders) {
    borders_ = borders;
}

void SwarmCtrl::setRepParam(const double repulsion) {
    rep_ = repulsion;
}

void SwarmCtrl::setAttractParam(const double attract) {
    atr_ = attract;
}

void SwarmCtrl::setMaxIteration(const int maxIter) {
    maxIter_ = maxIter;
}

std::vector<double> SwarmCtrl::getOptimalPosition(const std::vector<std::vector<double>> &initialPositionSwarm,
                                                  const std::vector<double> &yourPosition) {
    // Number of drones.
    unsigned int n = initialPositionSwarm.size() + 1;
    std::vector<double> U(2, 0), temp(4, 0);
    std::vector<std::vector<double>> swarm_coords(n);
    // Browse sample time
    for (int i = 0; i < maxIter_ - 1; ++i) {
        // Browse each drone in the swarm
        for (int j = 0; j < n; ++j) {
            // Compute drone repulsion
            for (int k = 0; k < n; ++k) {
                U += repulsive(swarm_coords[j], swarm_coords[k]);
            }
            // Border computation
            U += border(swarm_coords[j]);

            // System dynamic
            temp[0] = swarm_coords[i][0] + U[0]; //Vx
            temp[1] = swarm_coords[i][1] + U[1]; //Vy
            temp[2] = swarm_coords[i][2] + swarm_coords[i][0] * Te_; //X
            temp[3] = swarm_coords[i][3] + swarm_coords[i][1] * Te_; //Y
            swarm_coords[i] = temp;
        }
    }
    return std::vector<double>();
}

std::vector<double> SwarmCtrl::getLocalGradientDirection(std::vector<std::vector<double>> currentPosition,
                                                         std::vector<double> optimalPosition,
                                                         std::vector<double> yourPosition) {
    std::vector<double> U;
    U = repulsive(currentPosition, yourPosition);
    U += attractive(optimalPosition, yourPosition);
    return U;
}

std::vector<double> SwarmCtrl::attractive(std::vector<double> posOptimal, std::vector<double> currentPos) {
    return std::vector<double>();
}

std::vector<double> SwarmCtrl::border(std::vector<double> currentPos) {
    return std::vector<double>();
}

std::vector<double> SwarmCtrl::repulsive(std::vector<double> otherDrones, std::vector<double> currentPos) {
    return std::vector<double>();
}

std::vector<double> SwarmCtrl::repulsive(std::vector<std::vector<double>> otherDrones, std::vector<double> currentPos) {
    std::vector<double> temp;
    for (const auto &it : otherDrones) {
        temp += repulsive(it, currentPos);
    }
    return temp;
}

void SwarmCtrl::setSampleTime(double Te) {
    Te_ = Te;
}

