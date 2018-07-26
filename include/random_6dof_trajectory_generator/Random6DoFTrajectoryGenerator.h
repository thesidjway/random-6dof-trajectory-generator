#include <vector>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cmath>

class Random6DoFTrajectoryGenerator {
private:
public:
    Random6DoFTrajectoryGenerator();
    ~Random6DoFTrajectoryGenerator();
    void generateNRandomPoints(std::vector<std::vector<double>>& points, int n, int ROOM_SIZE);
    double bernstein(int n, int j, double t);
    void bezier(std::vector<std::vector<double>>& points, std::vector<double>& t, std::vector<std::vector<double>>& trajectory);
    void linspace(double n, std::vector<double>& t);
    void writeDataToFile(std::vector<std::vector<double>>& trajectory, std::string filename);
    void relativeRotationVec3sToQuat(std::vector<double> Vec1, std::vector<double> Vec2, std::vector<double>& output_Quat);
    inline unsigned int factorial(unsigned int n) {
        if (n == 1 || n == 0)
            return 1;
        else
            return n * factorial(n - 1);
    }
};