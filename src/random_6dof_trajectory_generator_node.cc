#include <random_6dof_trajectory_generator/Random6DoFTrajectoryGenerator.h>

int main(int argc, char** argv) {
    Random6DoFTrajectoryGenerator generator;
    std::vector<std::vector<double>> generated_points, trajectory;
    std::vector<double> t_vals;
    generator.generateNRandomPoints(generated_points, 8, 20);
    generator.linspace(5000, t_vals);
    generator.bezier(generated_points, t_vals, trajectory);
    generator.writeDataToFile(trajectory, "/home/thesidjway/trajectory.txt");
}
