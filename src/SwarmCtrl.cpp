#include <SwarmCtrl.hpp>
#include <cmath>
#include <utility>

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

std::vector<double> operator*(double gain, const std::vector<double> &v) {
    std::vector<double> temp(v.size());
    for (int i = 0; i < v.size(); ++i) {
        temp[i] = v[i] * gain;
    }
    return temp;
}

void SwarmCtrl::setBorders(const std::vector<double> &borders) {
    upper_boundary = borders[0];
    lower_boundary = borders[1];
    right_boundary = borders[2];
    left_boundary = borders[3];
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
                                                  const std::vector<double> &yourPosition) const {
    // Number of drones.
    unsigned int n = initialPositionSwarm.size() + 1;
    std::vector<double> U(2, 0), pos(2, 0), temp(4, 0);
    std::vector<std::vector<double>> swarm_coords(n, std::vector<double>(4, 0)), Uvec(n, std::vector<double>(2, 0));
    // Initialize
    for (int i = 0; i < n - 1; ++i) {
        swarm_coords[i][2] = initialPositionSwarm[i][0];
        swarm_coords[i][3] = initialPositionSwarm[i][1];
    }
    swarm_coords[n - 1][2] = yourPosition[0];
    swarm_coords[n - 1][3] = yourPosition[1];

    // Browse sample time
    for (int i = 0; i < maxIter_ - 1; ++i) {
        // Browse each drone in the swarm
        for (int j = 0; j < n; ++j) {
            U[0] = 0;
            U[1] = 0;
            // Compute drone repulsion
            for (int k = 0; k < n; ++k) {
                if (k == j) {
                    continue;
                }
                U += repulsive(swarm_coords[k], swarm_coords[j]);
            }
            // Border computation
            U += border(swarm_coords[j]);
            Uvec[j] = U;
        }
        // System dynamic update
        for (int j = 0; j < n; ++j) {
            temp[0] = -swarm_coords[j][0] + Uvec[j][0]; //Vx
            temp[1] = -swarm_coords[j][1] + Uvec[j][1]; //Vy
            temp[2] = swarm_coords[j][2] + swarm_coords[j][0] * Te_; //X
            temp[3] = swarm_coords[j][3] + swarm_coords[j][1] * Te_; //Y
            // Update vector
            swarm_coords[j] = temp;
        }
    }
    pos[0] = swarm_coords[n - 1][2];
    pos[1] = swarm_coords[n - 1][3];
    printVec(swarm_coords);
    return pos;
}

std::vector<double> SwarmCtrl::getLocalGradientDirection(const std::vector<std::vector<double>> &otherDrones,
                                                         std::vector<double> optimalPosition,
                                                         const std::vector<double> &currentPos) {
    std::vector<double> U(2, 0);
    U = repulsive(otherDrones, currentPos);
    U += border(currentPos);
    U += attractive(std::move(optimalPosition), currentPos);
    return U;
}

std::vector<double> SwarmCtrl::attractive(std::vector<double> posOptimal, std::vector<double> currentPos) {
    std::vector<double> temp;
    double norm = pow((currentPos[0] - posOptimal[0]), 2) + pow((currentPos[1] - posOptimal[1]), 2);
    temp.push_back((-currentPos[0] + posOptimal[0]) / norm);
    temp.push_back((-currentPos[1] + posOptimal[1]) / norm);
    return temp;
}

std::vector<double> SwarmCtrl::border(std::vector<double> currentPos) const {
    std::vector<double> temp(2);
    if (currentPos.size() == 4) {
        temp[0] = (exp(-currentPos[2] + left_boundary) - exp(currentPos[2] - right_boundary));
        temp[1] = (exp(-currentPos[3] + lower_boundary) - exp(currentPos[3] - upper_boundary));
    } else {
        temp[0] = (exp(-currentPos[0] + left_boundary) - exp(currentPos[0] - right_boundary));
        temp[1] = (exp(-currentPos[1] + lower_boundary) - exp(currentPos[1] - upper_boundary));
    }
    return temp;
}

std::vector<double> SwarmCtrl::repulsive(std::vector<double> otherDrones, std::vector<double> currentPos) {
    std::vector<double> temp(2, 0);
    double norm = pow(pow(currentPos[2] - otherDrones[2], 2) + pow(currentPos[3] - otherDrones[3], 2), 2);

    if (norm < 1e-15) {
        return std::vector<double>(2, 0);
    }
    temp[0] = (currentPos[2] - otherDrones[2]) / norm;
    temp[1] = (currentPos[3] - otherDrones[3]) / norm;
    return temp;
}

std::vector<double> SwarmCtrl::repulsive(const std::vector<std::vector<double>> &otherDrones,
                                         const std::vector<double> &currentPos) {
    std::vector<double> temp;
    for (const auto &it : otherDrones) {
        double norm = pow(pow(currentPos[0] - it[0], 2) + pow(currentPos[1] - it[1], 2), 2);
        if (norm < 1e-15) {
            continue;
        }
        temp[0] += (currentPos[0] - it[0]) / norm;
        temp[1] += (currentPos[1] - it[1]) / norm;
    }
    return temp;
}