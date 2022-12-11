/**
 * @file bullet.cpp
 * @author franklinselva (franklinselva10@gmail.com)
 * @brief A bullet api for atlas humanoid robot
 * @version 0.1
 * @date 2022-10-06
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <atlas/bullet/api.h>

namespace Simulators {
BulletApi::BulletApi() {
  btDefaultCollisionConfiguration *collisionConfiguration =
      new btDefaultCollisionConfiguration();
}

BulletApi::~BulletApi() {}
} // namespace Simulators