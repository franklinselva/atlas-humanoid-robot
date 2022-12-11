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

namespace Simulators
{

  BulletApi::BulletApi()
  {
    /// collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    mCollisionConfiguration_ = new btDefaultCollisionConfiguration();

    /// use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    mDispatcher_ = new btCollisionDispatcher(mCollisionConfiguration_);

    /// btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    mOverlappingPairCache_ = new btDbvtBroadphase();

    /// the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    mSolver_ = new btSequentialImpulseConstraintSolver;

    mDynamicsWorld_ = new btDiscreteDynamicsWorld(mDispatcher_, mOverlappingPairCache_, mSolver_, mCollisionConfiguration_);

    mDynamicsWorld_->setGravity(btVector3(0, -10, 0));

    // Add Ground
    BulletApi::setupGround();
  }

  BulletApi::~BulletApi()
  {
    // remove the rigidbodies from the dynamics world and delete them
    for (int i = mDynamicsWorld_->getNumCollisionObjects() - 1; i >= 0; i--)
    {
      btCollisionObject *obj = mDynamicsWorld_->getCollisionObjectArray()[i];
      btRigidBody *body = btRigidBody::upcast(obj);
      if (body && body->getMotionState())
      {
        delete body->getMotionState();
      }
      mDynamicsWorld_->removeCollisionObject(obj);
      delete obj;
    }

    // delete collision shapes
    for (int j = 0; j < mCollisionShapes_.size(); j++)
    {
      btCollisionShape *shape = mCollisionShapes_[j];
      mCollisionShapes_[j] = 0;
      delete shape;
    }

    // delete dynamics world
    delete mDynamicsWorld_;

    // delete solver
    delete mSolver_;

    // delete broadphase
    delete mOverlappingPairCache_;

    // delete dispatcher
    delete mDispatcher_;

    delete mCollisionConfiguration_;

    mCollisionShapes_.clear();
  }

  bool BulletApi::setupGround()
  {
    btCollisionShape *groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

    mCollisionShapes_.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -56, 0));

    btScalar mass(0.);

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
      groundShape->calculateLocalInertia(mass, localInertia);

    // using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState *myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
    btRigidBody *body = new btRigidBody(rbInfo);

    // add the body to the dynamics world
    mDynamicsWorld_->addRigidBody(body);

    return true;
  }

  bool BulletApi::addDefaultRigidBody(
      btCollisionShape *shape, btScalar mass = 1.f,
      btTransform startTransform = btTransform::getIdentity(),
      btVector4 color = btVector4(1, 1, 1, 1))
  {
    // create a dynamic rigidbody

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);
    btCollisionShape *colShape = new btSphereShape(btScalar(1.));
    mCollisionShapes_.push_back(colShape);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
      colShape->calculateLocalInertia(mass, localInertia);

    startTransform.setOrigin(btVector3(2, 10, 0));

    // using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState *myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
    btRigidBody *body = new btRigidBody(rbInfo);

    mDynamicsWorld_->addRigidBody(body);

    return true;
  }

  void BulletApi::stepSimulation()
  {
    mDynamicsWorld_->stepSimulation(1.f / 60.f, 10);

    // print positions of all objects
    for (int j = mDynamicsWorld_->getNumCollisionObjects() - 1; j >= 0; j--)
    {
      btCollisionObject *obj = mDynamicsWorld_->getCollisionObjectArray()[j];
      btRigidBody *body = btRigidBody::upcast(obj);
      btTransform trans;
      if (body && body->getMotionState())
      {
        body->getMotionState()->getWorldTransform(trans);
      }
      else
      {
        trans = obj->getWorldTransform();
      }
      printf("world pos object %d = %f,%f,%f\n", j, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
    }
  }
} // namespace Simulators