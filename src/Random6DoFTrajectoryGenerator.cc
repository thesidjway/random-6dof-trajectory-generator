#include <random_6dof_trajectory_generator/Random6DoFTrajectoryGenerator.h>

Random6DoFTrajectoryGenerator::Random6DoFTrajectoryGenerator() {
}

Random6DoFTrajectoryGenerator::~Random6DoFTrajectoryGenerator() {
	
}

double Random6DoFTrajectoryGenerator::bernstein ( int n, int j, double t) {
   return factorial(n)/(factorial(j)*factorial(n-j))*pow(t,j)*pow((1-t),(n-j));
}

void Random6DoFTrajectoryGenerator::linspace ( double n, std::vector< double >& t ) {
    for(double i = 0 ; i < 1; i+= 1.0/n) {
        t.push_back(i);
    }
}


void Random6DoFTrajectoryGenerator::bezier ( std::vector< std::vector< double > >& points, std::vector< double >& t, std::vector<std::vector<double>>& trajectory) {
  for (unsigned int i = 0; i < t.size(); i++) {
      std::vector<double> a;
      a.push_back(0);
      a.push_back(0);
      a.push_back(0);
      trajectory.push_back(a);
      for (unsigned int j = 0 ; j < points.size() ; j++) {
          for(unsigned int k = 0 ; k < 3; k++) {
            trajectory[i][k] = trajectory[i][k] + points[j][k] * bernstein(points.size(), j, t[i]);
          }
      }
  }
  for (unsigned int i = 0 ; i < trajectory.size() ; i ++ ) {
      std::cout << "[" << trajectory[i][0] << ", " << trajectory[i][1] << ", " << trajectory[i][2] << "]" << std::endl;
  }
}


void Random6DoFTrajectoryGenerator::generateNRandomPoints(std::vector<std::vector<double>>& points, int n, int ROOM_SIZE) {
    std::srand(time(NULL));
    for (unsigned int i = 0 ; i < n ; i++) {
        double temp1 = (-ROOM_SIZE * 1000 + rand()%(ROOM_SIZE * 2000))/1000.0;
        double temp2 = (-ROOM_SIZE * 1000 + rand()%(ROOM_SIZE * 2000))/1000.0;
        double temp3 = (-ROOM_SIZE * 1000 + rand()%(ROOM_SIZE * 2000))/1000.0;
        std::vector<double> a;
        a.push_back(temp1);
        a.push_back(temp2);
        a.push_back(temp3);
        points.push_back(a);
    }
}

void Random6DoFTrajectoryGenerator::relativeRotationVec3sToQuat ( std::vector< double > Vec1, std::vector< double > Vec2, std::vector< double >& output_Quat ) {
    double angle = acos((Vec1[0]*Vec2[0] + Vec1[1]*Vec2[1] + Vec1[2]*Vec2[2])/
    (sqrt(pow(Vec1[0], 2) + pow(Vec1[1], 2) + pow(Vec1[2], 2)) + sqrt(pow(Vec2[0], 2) + pow(Vec2[1], 2) + pow(Vec2[2], 2))));
    double s = sin(angle/2);
    double tmpx = Vec1[1] * Vec2[2] - Vec1[2] * Vec2[1];
    double tmpy = Vec1[0] * Vec2[2] - Vec1[2] * Vec2[0];
    double tmpz = Vec1[0] * Vec2[1] - Vec1[1] * Vec2[0];
    double axisx = tmpx/sqrt(pow(tmpx,2) + pow(tmpy,2) + pow(tmpz,2));
    double axisy = tmpy/sqrt(pow(tmpx,2) + pow(tmpy,2) + pow(tmpz,2));
    double axisz = tmpz/sqrt(pow(tmpx,2) + pow(tmpy,2) + pow(tmpz,2));
    output_Quat.push_back(axisx * s);
    output_Quat.push_back(axisy * s);
    output_Quat.push_back(axisz * s);
    output_Quat.push_back(cos(angle/2));
    std::cout << axisx * s << " " << axisy * s << " " << axisz * s << " " << cos(angle/2) << std::endl;
}


void Random6DoFTrajectoryGenerator::writeDataToFile ( std::vector< std::vector< double > >& trajectory, 
                                                      std::string filename ) {
    std::ofstream myfile;
    myfile.open(filename);
    std::vector<double> init;
    init.push_back(1);
    init.push_back(0);
    init.push_back(0); // 1i + 0j + 0k
    for(unsigned int i = 1; i < trajectory.size(); i++) {
        std::vector<double> tangent, quat;
        double tmpx = trajectory[i][0] - trajectory[i - 1][0];
        double tmpy = trajectory[i][1] - trajectory[i - 1][1];
        double tmpz = trajectory[i][2] - trajectory[i - 1][2];
        tangent.push_back(tmpx/sqrt(pow(tmpx,2) + pow(tmpy,2) + pow(tmpz,2)));
        tangent.push_back(tmpy/sqrt(pow(tmpx,2) + pow(tmpy,2) + pow(tmpz,2)));
        tangent.push_back(tmpz/sqrt(pow(tmpx,2) + pow(tmpy,2) + pow(tmpz,2)));
        relativeRotationVec3sToQuat(init, tangent, quat);
        myfile << "position: \n";
        myfile << "\tx: " << trajectory[i][0] << "\n";
        myfile << "\ty: " << trajectory[i][1] << "\n";
        myfile << "\tz: " << trajectory[i][2] << "\n";
        myfile << "orientation: \n";
        myfile << "\tx: " << quat[0] << "\n";
        myfile << "\ty: " << quat[1] << "\n";
        myfile << "\tz: " << quat[2] << "\n";
        myfile << "\tw: " << quat[3] << "\n";
        myfile << "---\n";
    }        
    myfile.close();
}
