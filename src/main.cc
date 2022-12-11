#include <atlas/bullet/api.h>
#include <iostream>
#include <stdio.h>

int main(int argc, char **argv) {
  btDefaultCollisionConfiguration *collisionConfiguration =
      new btDefaultCollisionConfiguration();

  btCollisionDispatcher *dispatcher =
      new btCollisionDispatcher(collisionConfiguration);

  btBroadphaseInterface *overlappingPairCache = new btDbvtBroadphase();

  btSequentialImpulseConstraintSolver *solver =
      new btSequentialImpulseConstraintSolver;

  btDiscreteDynamicsWorld *dynamicsWorld = new btDiscreteDynamicsWorld(
      dispatcher, overlappingPairCache, solver, collisionConfiguration);

  dynamicsWorld->setGravity(btVector3(0, -10, 0));

  for (int i = 0; i < 100; i++) {
    dynamicsWorld->stepSimulation(1 / 60.f, 10);

    for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--) {
      btCollisionObject *obj = dynamicsWorld->getCollisionObjectArray()[j];
      btRigidBody *body = btRigidBody::upcast(obj);
      btTransform trans;
      if (body && body->getMotionState()) {
        body->getMotionState()->getWorldTransform(trans);
      } else {
        trans = obj->getWorldTransform();
      }
      printf("world pos object %d = %f,%f,%f\n", j,
             float(trans.getOrigin().getX()), float(trans.getOrigin().getY()),
             float(trans.getOrigin().getZ()));
    }
  }

  // Remove all objects
  for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
    btCollisionObject *obj = dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody *body = btRigidBody::upcast(obj);
    if (body && body->getMotionState()) {
      delete body->getMotionState();
    }
    dynamicsWorld->removeCollisionObject(obj);
    delete obj;
  }

  // Delete dynamics world
  delete dynamicsWorld;

  // Delete solver
  delete solver;

  // Delete broadphase
  delete overlappingPairCache;

  // Delete dispatcher
  delete dispatcher;

  std::cout << "Hello, world!" << std::endl;
}

// Hello world
// int main(int argc, char **argv) {
//   printf("Hello world\n");
//   return 0;
// }