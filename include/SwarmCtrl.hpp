#ifndef SWARMCTRL_SWARMCTRL_HPP
#define SWARMCTRL_SWARMCTRL_HPP

#include <vector>
#include <iostream>

void printVec(const std::vector<double> &v);

void printVec(const std::vector<std::vector<double>> &v);

std::vector<double> operator+(const std::vector<double> &v1, const std::vector<double> &v2);

std::vector<double> operator-(const std::vector<double> &v1, const std::vector<double> &v2);

std::vector<double> operator+=(std::vector<double> &v1, const std::vector<double> &v2);

std::vector<double> operator*(double gain, const std::vector<double> &v2);

class FieldGradientEquations {
public:

    static std::vector<double> attractive(std::vector<double> posOptimal,
                                          std::vector<double> currentPos);

    static std::vector<double> boundary(std::vector<double> currentPos, std::vector<double> boundary);

    static std::vector<double> repulsive(std::vector<double> otherDrones,
                                         std::vector<double> currentPos);

    static std::vector<double> repulsive(const std::vector<std::vector<double>> &otherDrones,
                                         const std::vector<double> &currentPos);

};

class SwarmCtrl : public FieldGradientEquations {
public:

    void setBorders(const std::vector<double> &borders);

    void setRepParam(double repulsion);

    void setMaxIteration(int maxIter);

    std::vector<double> getOptimalPosition(const std::vector<std::vector<double>> &initialPositionSwarm,
                                           const std::vector<double> &yourPosition) const;

    std::vector<double> getLocalGradientDirection(const std::vector<std::vector<double>> &otherDrones,
                                                  std::vector<double> optimalPosition,
                                                  const std::vector<double> &currentPos);

protected:

    int maxIter_{};
    double left_boundary{}, right_boundary{}, upper_boundary{}, lower_boundary{}, rep_{};
    std::vector<double> boundary_limit{};

    static inline double adaptiveRate(double a);

    inline double vecNorm(const std::vector<double> &vec) const;

    inline std::vector<double> sysUpdate(std::vector<double> X, std::vector<double> U) const;
};

#endif //SWARMCTRL_SWARMCTRL_HPP
