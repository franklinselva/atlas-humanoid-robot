/**
 * @file api.hpp
 * @author franklinselva (franklinselva10@gmail.com)
 * @brief A bullet api for atlas humanoid robot
 * @version 0.1
 * @date 2022-10-06
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef ATLAS_BULLET_API_HPP
#define ATLAS_BULLET_API_HPP

#include <string>
#include <tuple>
#include <Bullet3Common/b3HashMap.h>

namespace Simulators {
class BulletApi {
public:
  BulletApi();
  ~BulletApi();

  // Load a robot
  bool loadRobot(std::string robotFile, std::string robotName,
                 bool isFixed = false);
  bool loadRobot(std::string robotFile, std::string robotName,
                 std::tuple<double, double, double> position = {0, 0, 0},
                 std::tuple<double, double, double> orientation = {0, 0, 0},
                 bool isFixed = false);

  // Load Environment
  bool loadEnvironment(std::string environmentFile, std::string environmentName,
                       bool isFixed = false);

private:
  std::string mRobotFile_;
  std::string mRobotName_;
  std::string mEnvironmentFile_;
  std::string mEnvironmentName_;

  /* load data */
  void _load_urdf(std::string urdfFile, std::string modelName,
                  std::tuple<double, double, double> position = {0, 0, 0},
                  std::tuple<double, double, double> orientation = {0, 0, 0},
                  bool isFixed = false);

  void _load_sdf(std::string sdfFile, std::string modelName,
                 std::tuple<double, double, double> position = {0, 0, 0},
                 std::tuple<double, double, double> orientation = {0, 0, 0},
                 bool isFixed = false);
};

};     // namespace Simulators
#endif // ATLAS_BULLET_API_HPP