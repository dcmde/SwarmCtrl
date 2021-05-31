#ifndef SWARMCTRL_SWARMCTRL_HPP
#define SWARMCTRL_SWARMCTRL_HPP

class SwarmCtrl {
public:
    void setBorders(std::vector<double>);

    void setRepParam(double);

    void setAttractParam(double);

    void setMaxIteration(int maxIter);

    std::vector<double> getOptimalPosition(std::vector <std::vector<double>> initialPositionSwarm,
                                           std::vector<double> yourPosition);

    std::vector<double> getLocalGradientDirection(std::vector <std::vector<double>> currentPosition,
                                                  std::vector<double> yourPosition,
                                                  bool= true);

protected:

    inline std::vector<double> attractive(std::vector<double>);

    inline std::vector<double> border(std::vector<double>);

    inline std::vector<double> repulsive(std::vector<double>);
};

#endif //SWARMCTRL_SWARMCTRL_HPP
