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
#ifndef ATLAS_BULLET_API_H
#define ATLAS_BULLET_API_H

#include "btBulletDynamicsCommon.h"

#include <string>
#include <assert.h>
#include <stdio.h>
#include <tuple>

#ifdef USE_GUI
#include "LinearMath/btIDebugDraw.h"
#endif

#define ASSERT_EQ(a, b) assert((a) == (b));

namespace Simulators
{
  class BulletApi
  {
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
    /* Bullet Physics Default ground*/
    bool setupGround();
    bool addDefaultRigidBody(btCollisionShape *shape, btScalar mass,
                             btTransform startTransform,
                             btVector4 color);

    void stepSimulation();

  private:
    std::string mRobotFile_;
    std::string mRobotName_;
    std::string mEnvironmentFile_;
    std::string mEnvironmentName_;

    // Bullet Physics
    btDefaultCollisionConfiguration *mCollisionConfiguration_;
    btCollisionDispatcher *mDispatcher_;
    btBroadphaseInterface *mOverlappingPairCache_;
    btSequentialImpulseConstraintSolver *mSolver_;
    btDiscreteDynamicsWorld *mDynamicsWorld_;
    btAlignedObjectArray<btCollisionShape *> mCollisionShapes_;

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
#endif // ATLAS_BULLET_API_H