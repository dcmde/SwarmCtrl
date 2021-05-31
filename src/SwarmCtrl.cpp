#include <SwarmCtrl.hpp>

void printVec(const std::vector<double> &v) {
    for (auto it : v) {
        std::cout << it << " ";
    }
    std::cout << std::endl;
}

void printVec(const std::vector<std::vector<double>> &v) {
    for (const auto &it : v) {
        for (const auto &el : it) {
            std::cout << el << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void SwarmCtrl::setSampleTime(double Te) {
    Te_ = Te;
}

std::vector<double> operator+(const std::vector<double> &v1, const std::vector<double> &v2) {
    std::vector<double> temp;
    if (v1.size() != v2.size()) {
        return temp;
    }
    for (int i = 0; i < v1.size(); ++i) {
        temp.push_back(v1[i] + v2[i]);
    }
    return temp;
}

std::vector<double> operator+=(std::vector<double> &v1, const std::vector<double> &v2) {
    std::vector<double> temp;
    if (v1.size() != v2.size()) {
        return temp;
    }
    for (int i = 0; i < v1.size(); ++i) {
        temp.push_back(v1[i] + v2[i]);
    }
    v1 = temp;
    return temp;
}

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
    std::vector<double> U(2, 0), pos(2, 0), temp(4, 0);
    std::vector<std::vector<double>> swarm_coords(n, std::vector<double>(2, 0));
    // Initialize
    for (int i = 0; i < n - 1; ++i) {
        swarm_coords[i] = initialPositionSwarm[i];
    }
    swarm_coords[n - 1] = yourPosition;

    // Browse sample time
    for (int i = 0; i < maxIter_ - 1; ++i) {
        // Browse each drone in the swarm
        for (int j = 0; j < n; ++j) {
            // Compute drone repulsion
            for (int k = 0; k < n; ++k) {
//                U += repulsive(swarm_coords[j], swarm_coords[k]);
            }
            // Border computation
//            U += border(swarm_coords[j]);

            // System dynamic update
            temp[0] = swarm_coords[j][0] + U[0]; //Vx
            temp[1] = swarm_coords[j][1] + U[1]; //Vy
            temp[2] = swarm_coords[j][2] + swarm_coords[j][0] * Te_; //X
            temp[3] = swarm_coords[j][3] + swarm_coords[j][1] * Te_; //Y
            // Update vector
            swarm_coords[j] = temp;
        }
    }
    pos[0] = swarm_coords[n - 1][2];
    pos[1] = swarm_coords[n - 1][3];
    return std::vector<double>({0,0});
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
    return std::vector<double>({0,0});
}

std::vector<double> SwarmCtrl::border(std::vector<double> currentPos) {
    return std::vector<double>({0,0});
}

std::vector<double> SwarmCtrl::repulsive(std::vector<double> otherDrones, std::vector<double> currentPos) {
    return std::vector<double>({0,0});
}

std::vector<double> SwarmCtrl::repulsive(std::vector<std::vector<double>> otherDrones, std::vector<double> currentPos) {
    std::vector<double> temp;
    for (const auto &it : otherDrones) {
        temp += repulsive(it, currentPos);
    }
    return temp;
}

