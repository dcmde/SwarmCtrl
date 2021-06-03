#include <SwarmCtrl.hpp>
#include <fstream>
#include <sstream>

void split(const std::string &str, std::vector<double> &vec, char delim = ' ') {
    double val;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delim)) {
        val = std::stod(token);
        vec.push_back(val);
    }
}

int main(int argc, char *argv[]) {
    std::vector<std::vector<double>> vecInit;
    std::vector<double> vecTemp, uPos;
    SwarmCtrl swarmCtrl;

    int maxIter, n;
    double repParam, sampleTime, upper, lower, right, left;


//    swarmCtrl.getOptimalPosition(vec, uPos);
//
//    printVec(swarmCtrl.getOptimalPosition(vec, uPos));

    std::ofstream ofstream("swarmCoords.txt");
    std::string line;

    if (argc != 7) {
        std::cout << "Enter : maxIter repParam upper lower right left" << std::endl;
        return 0;
    }

    std::ifstream ifstream("swarmInitPos.txt");

    if (!ifstream.is_open()) {
        std::cout << "Cannot find swarmInitPos.txt" << std::endl;
        return 0;
    }

    maxIter = std::stoi(argv[1]);
    repParam = std::stod(argv[2]);
    upper = std::stod(argv[3]);
    lower = std::stod(argv[4]);
    right = std::stod(argv[5]);
    left = std::stod(argv[6]);

    swarmCtrl.setRepParam(repParam);
    swarmCtrl.setMaxIteration(maxIter);
    swarmCtrl.setBorders(std::vector<double>({upper, lower, right, left}));

    while (std::getline(ifstream, line)) {
        vecTemp.clear();
        split(line, vecTemp);
        vecInit.push_back(vecTemp);
    }
    uPos = vecInit[0];
    vecInit.erase(vecInit.begin());
    n = vecInit.size();

    printVec(swarmCtrl.getOptimalPosition(vecInit, uPos));

    return 0;
}
