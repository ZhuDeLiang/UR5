#include <Eigen/Geometry>

#include "butterworth.h"
#include "spatial_utilities.h"
#include "timer_linux.h"

using namespace RUT;

int main() {

  // create a butterworth filter with cutoff frequency of 50 rad/s
  // and sampling time of 1/200 s
  // und 2 input channels filtered in parallel
  Butterworth filter{50, 1.0 / 200, 4, 2};
  Butterworth filter_eigen{50, 1.0 / 200, 4, 2};
  std::vector<double> u{4, 3};
  Eigen::Vector2d u_eigen;
  u_eigen << 4, 3;

  for (int i = 0; i < 100; i++) {
    std::vector<double> y = filter.step(u);
    Eigen::VectorXd y_eigen = filter_eigen.step(u_eigen);
    // print output
    std::cout << y[0] << ", " << y[1] << "," << y_eigen[0] << ", " << y_eigen[1]
              << std::endl;
  }

  Matrix3d rotation = rotX(0.6) * rotY(0.3) * rotZ(0.2);
  Vector3d translation;
  translation << 1, 2, 3;

  Matrix4d SE3_1 = Matrix4d::Identity();
  SE3_1.block<3, 3>(0, 0) = rotation;
  SE3_1.block<3, 1>(0, 3) = translation;

  Vector3d delta_translation;
  Matrix3d delta_rotation;
  Matrix4d SE3_2 = Matrix4d::Identity();
  Matrix4d SE3_delta = Matrix4d::Identity();

  // // test: difference between twist, spt, and vel
  // for (int i = 0; i < 50; i++) {
  //   double delta = exp(-i * 2 / 10.0);

  //   delta_translation << delta, 0, 0;
  //   delta_rotation = rotX(delta) * rotY(delta);  // * rotZ(delta);
  //   SE3_delta.block<3, 3>(0, 0) = delta_rotation;
  //   SE3_delta.block<3, 1>(0, 3) = delta_translation;

  //   SE3_2 = SE3_1 * SE3_delta;

  //   Vector6d twist = SE32se3(SE3_delta);
  //   Vector6d spt = SE32spt(SE3_delta);
  //   double dt = 1;
  //   Vector6d vel = vee6(SE3Inv(SE3_1) * (SE3_2 - SE3_1) / dt);
  //   // Vector6d vel = vee6((SE3_2 - SE3_1) / dt * SE3Inv(SE3_1)); // spatial velocity

  //   std::cout << "delta: \t" << delta << std::endl;
  //   std::cout << "twist.normalized(): \t" << twist.normalized().transpose()
  //             << std::endl;
  //   std::cout << "spt.normalized(): \t" << spt.normalized().transpose()
  //             << std::endl;
  //   std::cout << "vel.normalized(): \t" << vel.normalized().transpose()
  //             << std::endl;
  // }

  // test: integrate the twist between two SE3 as velocity
  delta_translation << 1, 2, 3;
  delta_rotation = rotX(1) * rotY(2) * rotZ(3);
  SE3_delta.block<3, 3>(0, 0) = delta_rotation;
  SE3_delta.block<3, 1>(0, 3) = delta_translation;

  SE3_2 = SE3_1 * SE3_delta;

  Vector6d twist = SE32se3(SE3_delta);
  Vector6d spt = SE32spt(SE3_delta);

  Matrix4d SE3_new = SE3_1;
  int N = 2000;
  double dt = 0.001;
  double distance = 0;
  for (int i = 0; i < N; i++) {
    SE3_new += dt * SE3_new * wedge6(spt);
    distance += dt * spt.norm();

    double dp = (SE3_new.block<3, 1>(0, 3) - SE3_2.block<3, 1>(0, 3)).norm();
    double dR = (SE3_new.block<3, 3>(0, 0) - SE3_2.block<3, 3>(0, 0)).norm();

    std::cout << "distance: \t" << distance << ", dp: " << dp << ", dR: " << dR
              << std::endl;
  }

 

  return 0;
}