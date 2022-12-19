#include <atlas/log.h>
#include <atlas/bullet/api.h>
#include <iostream>
#include <stdio.h>

using namespace Simulators;
using namespace HumanoidRobot;

int main(int argc, char **argv)
{
  BulletApi bulletApi;
  Log LOG;
  LOG.init();

  bulletApi.setupGround();
  btCollisionShape *shape = new btBoxShape(btVector3(1, 1, 1));

  btScalar mass = 1.0;
  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(btVector3(0, 0, 0));
  btVector4 color = {1, 1, 1, 1};

  bulletApi.addDefaultRigidBody(shape, mass, startTransform, color);

  LOG.info("Hello, world!");
  for (int i = 0; i < 300; i++)
  {
    bulletApi.stepSimulation();
  }
  LOG_INFO("Hello, world!");
}