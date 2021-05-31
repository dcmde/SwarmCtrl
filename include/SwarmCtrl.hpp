#ifndef SWARMCTRL_SWARMCTRL_HPP
#define SWARMCTRL_SWARMCTRL_HPP

#include <vector>

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

class SwarmCtrl {
public:
    void setSampleTime(double Te);

    void setBorders(const std::vector<double> &borders);

    void setRepParam(double repulsion);

    void setAttractParam(double attract);

    void setMaxIteration(int maxIter);

    std::vector<double> getOptimalPosition(const std::vector<std::vector<double>> &initialPositionSwarm,
                                           const std::vector<double> &yourPosition);

    std::vector<double> getLocalGradientDirection(std::vector<std::vector<double>> currentPosition,
                                                  std::vector<double> optimalPosition,
                                                  std::vector<double> yourPosition);

protected:

    int maxIter_{};
    double rep_{}, atr_{}, Te_{};
    std::vector<double> borders_{};

    inline std::vector<double> sys(const std::vector<double> &X, const std::vector<double> &U);

    inline std::vector<double> attractive(std::vector<double> posOptimal,
                                          std::vector<double> currentPos);

    inline std::vector<double> border(std::vector<double> currentPos);

    inline std::vector<double> repulsive(std::vector<double> otherDrones,
                                         std::vector<double> currentPos);

    inline std::vector<double> repulsive(std::vector<std::vector<double>> otherDrones,
                                         std::vector<double> currentPos);
};

#endif //SWARMCTRL_SWARMCTRL_HPP
