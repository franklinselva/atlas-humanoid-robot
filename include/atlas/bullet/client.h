/**
 * @file client.h
 * @author franklinselva (franklinselva10@gmail.com)
 * @brief Bullet3 Robot Simulator Client.
 * @version 0.1
 * @date 2022-10-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef ATLAS_BULLET_CLIENT_H_
#define ATLAS_BULLET_CLIENT_H_

#include <string>

namespace Simulators {
class BulletClient {
public:
  BulletClient();
  ~BulletClient();
  bool connect(int mode, const std::string &host = "localhost", int port = -1);
};

} // namespace Simulators
#endif // ATLAS_BULLET_CLIENT_H_