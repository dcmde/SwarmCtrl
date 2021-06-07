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

std::vector<double> FieldGradientEquations::attractive(std::vector<double> posOptimal, std::vector<double> currentPos) {
    std::vector<double> temp;
    temp.push_back((-currentPos[0] + posOptimal[0]));
    temp.push_back((-currentPos[1] + posOptimal[1]));
    return temp;
}

std::vector<double> FieldGradientEquations::boundary(std::vector<double> currentPos,
                                                     std::vector<double> boundary) {
    double upper_boundary, right_boundary, left_boundary, lower_boundary;
    std::vector<double> temp(2);

    upper_boundary = boundary[0];
    lower_boundary = boundary[1];
    right_boundary = boundary[2];
    left_boundary = boundary[3];

    if (currentPos.size() == 4) {
        temp[0] = (exp(-currentPos[2] + left_boundary) - exp(currentPos[2] - right_boundary));
        temp[1] = (exp(-currentPos[3] + lower_boundary) - exp(currentPos[3] - upper_boundary));
    } else {
        temp[0] = (exp(-currentPos[0] + left_boundary) - exp(currentPos[0] - right_boundary));
        temp[1] = (exp(-currentPos[1] + lower_boundary) - exp(currentPos[1] - upper_boundary));
    }
    return temp;
}

std::vector<double> FieldGradientEquations::repulsive(std::vector<double> otherDrones, std::vector<double> currentPos) {
    std::vector<double> temp(2, 0);
    double norm = pow(pow(currentPos[2] - otherDrones[2], 2) + pow(currentPos[3] - otherDrones[3], 2), 2);

    if (norm < 1e-15) {
        throw "repulsive divide by 0";
    }
    temp[0] = (currentPos[2] - otherDrones[2]) / norm;
    temp[1] = (currentPos[3] - otherDrones[3]) / norm;
    return temp;
}

std::vector<double> FieldGradientEquations::repulsive(const std::vector<std::vector<double>> &otherDrones,
                                                      const std::vector<double> &currentPos) {
    std::vector<double> temp(2, 0);
    printVec(otherDrones);
    printVec(currentPos);
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

std::vector<double> operator-(const std::vector<double> &v1, const std::vector<double> &v2) {
    std::vector<double> temp;
    if (v1.size() != v2.size()) {
        return temp;
    }
    for (int i = 0; i < v1.size(); ++i) {
        temp.push_back(v1[i] - v2[i]);
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

std::vector<double> operator/(const std::vector<double> &v, double gain) {
    std::vector<double> temp(v.size());
    for (int i = 0; i < v.size(); ++i) {
        temp[i] = v[i] / gain;
    }
    return temp;
}

void SwarmCtrl::setBorders(const std::vector<double> &borders) {
    upper_boundary = borders[0];
    lower_boundary = borders[1];
    right_boundary = borders[2];
    left_boundary = borders[3];
    boundary_limit.clear();
    boundary_limit.push_back(upper_boundary);
    boundary_limit.push_back(lower_boundary);
    boundary_limit.push_back(right_boundary);
    boundary_limit.push_back(left_boundary);
}

void SwarmCtrl::setRepParam(double repulsion) {
    rep_ = repulsion;
}

void SwarmCtrl::setMaxIteration(const int maxIter) {
    maxIter_ = maxIter;
}

std::vector<double> SwarmCtrl::getOptimalPosition(const std::vector<std::vector<double>> &initialPositionSwarm,
                                                  const std::vector<double> &yourPosition) const {
    // Number of drones.
    unsigned int n = initialPositionSwarm.size() + 1;
    std::vector<double> U(2, 0), pos(2, 0), temp(4, 0), Urep(2, 0), Ubnd(2, 0);
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
            Urep[0] = 0;
            Urep[1] = 0;
            Ubnd[0] = 0;
            Ubnd[1] = 0;
            // Compute drone repulsion
            for (int k = 0; k < n; ++k) {
                if (k == j) {
                    continue;
                }
                Urep += rep_ * repulsive(swarm_coords[k], swarm_coords[j]);
            }
            // Border computation
            Ubnd += boundary(swarm_coords[j], boundary_limit);
            U = Urep + Ubnd;
            Uvec[j] = adaptiveRate(vecNorm(U)) * U;
        }
        // System dynamic update
        for (int j = 0; j < n; ++j) {
            auto X = sysUpdate(swarm_coords[j], Uvec[j]);
            if (std::abs(swarm_coords[j][2] - X[2]) > 50 or std::abs(swarm_coords[j][3] - X[3]) > 50) {
                throw "Optimization diverge";
            }
//            } else if (vecNorm(Uvec[j]) < 1e-5) {
//                pos[0] = swarm_coords[n - 1][2];
//                pos[1] = swarm_coords[n - 1][3];
//                printVec(swarm_coords);
//                return pos;
//            }
            swarm_coords[j] = X;
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
    U += boundary(currentPos, boundary_limit);
    U += attractive(std::move(optimalPosition), currentPos);
    return U;
}

double SwarmCtrl::adaptiveRate(double norm) {
    double S = 1e-4;
    if (norm < S) {
        return 1 / S;
    }
    return 1 / norm;
}

double SwarmCtrl::vecNorm(const std::vector<double> &vec) const {
    double norm = 0;
    for (const auto &it : vec) {
        norm += pow(it, 2);
    }
    return sqrt(norm);
}

std::vector<double> SwarmCtrl::sysUpdate(std::vector<double> X, std::vector<double> U) const {
    std::vector<double> temp(4, 0);
    temp[0] = 0.9 * X[0] + 0.1 * U[0]; //Vx
    temp[1] = 0.9 * X[1] + 0.1 * U[1]; //Vy
    temp[2] = X[2] + 0.1 * X[0]; //X
    temp[3] = X[3] + 0.1 * X[1]; //Y
    return temp;
}
