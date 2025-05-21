#include <robot.h>

#include <iostream>

/*
 * This function shows an example for computing the WS of a desired robot,
 * provided an URDF file
 */

void SaveData(std::ofstream &File, Eigen::VectorXd Values);

int main(int argc, char **argv) {
  std::string urdf_file =
      "/home/holiday/ws_ws/src/WSRender_V0.0/src/resources/default.urdf";

  std::cout << "urdf_file: " << urdf_file << std::endl;

  // set limits
  std::vector<Eigen::VectorXd> JPos_lims(2);
  std::vector<Eigen::VectorXd> JTorque_lims(2);

  JPos_lims[0].resize(7);
  JPos_lims[1].resize(7);
  JTorque_lims[0].resize(7);
  JTorque_lims[1].resize(7);

  JPos_lims[0] << -3.142, -0.05, -1.0, -0.01, -1.5, -0.9, -0.7;
  JPos_lims[1] << 3.142, 3.142, 1.0, 2.0944, 1.5, 0.9, 0.7;

  JTorque_lims[0] << -300, -300, -300, -300, -300, -300, -300;
  JTorque_lims[1] << -JTorque_lims[0];

  std::vector<std::string> first_last_link_name(2);

  first_last_link_name[0] = "link0";
  first_last_link_name[1] = "link7";

  Eigen::Vector3d grav;  // gravity vector in base frame
  grav << 0., 0, -9.81;

  Robot Robot(urdf_file, first_last_link_name, grav);

  Robot.setLims(JPos_lims, JTorque_lims);

  int n_samples = 6;  // number of samples for joint sampling
  std::vector<std::vector<double>> space_bounds(2);  // bounds for 3D space

  space_bounds[0] = {-0.8, -0.8, -1.0};
  space_bounds[1] = {0.8, 0.8, 1.0};

  std::vector<Eigen::VectorXd> WS =
      Robot.getWorkspace(n_samples, space_bounds, 0.2);

  std::ofstream WS_file("my_WS_file_4.txt");

  std::cout << "saving \n";
  double X_min = -999999.9;
  double X_max = 999999.9;
  double Y_min = -999999.9;
  double Y_max = 999999.9;
  double Z_min = -9999999.9;
  double Z_max = 9999999.9;
  for (int i = 0; i < WS.size(); i++) {
    SaveData(WS_file, WS[i]);
    double X_temp = WS[i][0];
    double Y_temp = WS[i][1];
    double Z_temp = WS[i][2];
    if (X_temp < X_min)
      X_min = X_temp;
    else if (X_temp > X_max)
      X_max = X_temp;

    if (Y_temp < Y_min)
      Y_min = Y_temp;
    else if (Y_temp > Y_max)
      Y_max = Y_temp;

    if (Z_temp < Z_min)
      Z_min = Z_temp;
    else if (Z_temp > Z_max)
      Z_max = Z_temp;
  }
  std::cout << "X_min: " << X_min << std::endl;
  std::cout << "X_max: " << X_max << std::endl;
  std::cout << "Y_min: " << Y_min << std::endl;
  std::cout << "Y_max: " << Y_max << std::endl;
  std::cout << "Z_min: " << Z_min << std::endl;
  std::cout << "Z_max: " << Z_max << std::endl;

  WS_file.close();

  return 0;
}

void SaveData(std::ofstream &File, Eigen::VectorXd Values) {
  if (File.is_open()) {
    for (int i = 0; i < Values.rows(); i++) {
      File << Values[i] << "\t";
    }
    File << "\n";
  }
  return;
}
