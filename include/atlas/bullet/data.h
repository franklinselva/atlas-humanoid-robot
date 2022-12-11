/**
 * @file data.h
 * @author franklinselva (franklinselva10@gmail.com)
 * @brief Bullet data structures.
 * @version 0.1
 * @date 2022-10-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef ATLAS_BULLET_DATA_H_
#define ATLAS_BULLET_DATA_H_

#include <bullet/LinearMath/btAlignedObjectArray.h>
#include <bullet/LinearMath/btQuaternion.h>
#include <bullet/LinearMath/btTransform.h>
#include <bullet/LinearMath/btVector3.h>
#include <string>

namespace Simulators {

namespace {
struct RobotURDFSetup {
  std::string urdf;
  btVector3 base_position;
  btQuaternion base_orientation;

  RobotURDFSetup()
      : urdf(""), base_position(btVector3(0, 0, 0)),
        base_orientation(btQuaternion(0, 0, 0, 1)) {}

  RobotURDFSetup(const std::string &urdf, const btVector3 &base_position,
                 const btQuaternion &base_orientation)
      : urdf(urdf), base_position(base_position),
        base_orientation(base_orientation) {}
};

} // namespace
} // namespace Simulators
#endif // ATLAS_BULLET_DATA_H_