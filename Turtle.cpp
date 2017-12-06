#include "PhysicsGame1.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

#include "PhysicsFactory.h"
#include "Game.h" 
#include "Model.h"
#include "dirent.h"
#include "Capsule.h" 

#include "Turtle.h"

using namespace BGE;

Turtle::Turtle(void)
{
}

Turtle::~Turtle(void)
{
}


bool Turtle::Initialise()
{
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();
	physicsFactory->dynamicsWorld->setGravity(btVector3(0, -9, 0));

	//add wall and ball to scene
	physicsFactory->CreateWall(glm::vec3(20, 0, 20), 10, 10, 2, 2, 2);
	ball = physicsFactory->CreateSphere(2, glm::vec3(-20, 0, 20), glm::quat());

	shared_ptr<PhysicsController> turtle = CreateTurtle(glm::vec3(-10, 5, 10), 5);
	timeBeforeFlippingShell = FlipInterval;
	return Game::Initialise();
}

void BGE::Turtle::Update(float deltaTime)
{
	glm::vec3 bodyRotation = base->transform->up;
	/* Flip shell when on back*/
	//if upside, start timer before flipping shell
	if (bodyRotation.y < 0)
	{
		timeBeforeFlippingShell -= deltaTime;
	}
	//if right-side up, reset timer
	else
	{
		timeBeforeFlippingShell = FlipInterval;
	}

	//if flip timer has expired, flip shell and reset timer
	if (timeBeforeFlippingShell < 0)
	{
		FlipShell();
		timeBeforeFlippingShell = FlipInterval;
	}

	/* Wag tail when near to ball*/
	float distanceToBall = glm::distance(base->transform->position, ball->transform->position);
	if (distanceToBall <= 10)
	{
		//rotate tail
		tail->rigidBody->applyTorque(legTorque);
	}

	/* Head tracking ball*/
	glm::vec3 distVec = ball->transform->position - head->transform->position;
	float dot = glm::dot(glm::normalize(distVec), head->transform->look);
	PrintFloat("Dot Prduct: ", dot);

	float angle = glm::acos(dot);
	PrintFloat("Angle: ", angle);

	float degrees = glm::degrees(angle);
	PrintFloat("Degrees: ", degrees);

	/*if (dot < 0)
	{
		head->rigidBody->applyForce(btVector3(-1, 0, 0), btVector3(headScale, 0, 0));
	}
	else if ()
	{
		head->rigidBody->applyForce(btVector3(1, 0, 0), btVector3(-headScale, 0, 0));
	}*/

	//add force to move legs
	rightFrontLeg->rigidBody->applyTorque(legTorque);
	leftFrontLeg->rigidBody->applyTorque(legTorque);
	rightBackLeg->rigidBody->applyTorque(legTorque);
	leftBackLeg->rigidBody->applyTorque(legTorque);

	Game::Update(deltaTime);
}

void BGE::Turtle::Cleanup()
{
	Game::Cleanup();
}

shared_ptr<PhysicsController> Turtle::CreateTurtle(glm::vec3 position, float scale)
{
	shellRadius = scale / 2;
	//create body which all parts will attach to
	base = physicsFactory->CreateCylinder(shellRadius, shellRadius / 2, position, glm::quat());
	
	//create head
	headScale = scale / 6;
	head = physicsFactory->CreateSphere(headScale, position + glm::vec3(0, shellRadius / 4, shellRadius + 1), glm::quat());

	//connect head with body
	btHingeConstraint * headToBodyHinge = new btHingeConstraint(*base->rigidBody, *head->rigidBody,
		GLToBtVector(glm::vec3(0, shellRadius / 4, shellRadius)), GLToBtVector(glm::vec3(0, 0, -(headScale)-1)),
		btVector3(0, 1, 0), btVector3(0, 1, 0));

	dynamicsWorld->addConstraint(headToBodyHinge);
	
	//create tail
	tail = physicsFactory->CreateCapsule(scale / 20, scale / 18, position - glm::vec3(0, -(shellRadius / 4), shellRadius + scale / 20), glm::quat());
	
	//connect tail to body
	btTransform t13, t14;
	t13.setIdentity();
	t14.setIdentity();
	t13.getBasis().setEulerZYX(0, 0, M_PI);
	t13.setOrigin(btVector3(0, shellRadius / 4, -(shellRadius) - (scale / 10)));
	t14.getBasis().setEulerZYX(0, 0, M_PI);
	t14.setOrigin(btVector3(0, (scale / 20), 0));
	btConeTwistConstraint * tailToBodyJoint = new btConeTwistConstraint(*base->rigidBody, *tail->rigidBody, t13, t14);
	tailToBodyJoint->setLimit(glm::half_pi<float>(), glm::half_pi<float>(), 0);
	dynamicsWorld->addConstraint(tailToBodyJoint);
	
	//create legs and attach them
	double legLength = scale / 10;
	double legWidth = scale / 18;
	
	rightFrontLeg = CreateAndAttachLeg(position + glm::vec3(-(scale / 2) - legLength, -(legLength / 2), (scale / 3)), legWidth, legLength, (scale / 2), -(legLength / 2), -(scale / 3), base);
	leftFrontLeg = CreateAndAttachLeg(position + glm::vec3((scale / 2) + legLength, -(legLength / 2), (scale / 3)), legWidth, legLength, -(scale / 2), -(legLength / 2), -(scale / 3), base);
	rightBackLeg = CreateAndAttachLeg(position + glm::vec3(-(scale / 2) - legLength, -(legLength / 2), scale / 3), legWidth, legLength, (scale / 2), -(legLength / 2), scale / 3, base);
	leftBackLeg = CreateAndAttachLeg(position + glm::vec3((scale / 2) + legLength, -(legLength / 2), scale / 3), legWidth, legLength, -(scale / 2), -(legLength / 2), scale / 3, base);
	
	//create shell
	shell = physicsFactory->CreateCylinder(shellRadius, shellRadius / 2, position + glm::vec3(0, shellRadius / 2, 0), glm::quat());

	btHingeConstraint * shellHinge = new btHingeConstraint(*shell->rigidBody, *base->rigidBody,
		GLToBtVector(glm::vec3(0, -(shellRadius / 4), -(shellRadius))),	GLToBtVector(glm::vec3(0, shellRadius / 4, -(shellRadius))), 
		btVector3(1, 0, 0), btVector3(1, 0, 0));
	dynamicsWorld->addConstraint(shellHinge);

	//add latch to shell so it wont open while moving normally
	btTransform t7, t8;
	t7.setIdentity();
	t8.setIdentity();
	t7.setOrigin(btVector3(0, -(shellRadius / 4), shellRadius));
	t8.setOrigin(btVector3(0, shellRadius / 4, shellRadius));
	shellLatch = new btFixedConstraint(*shell->rigidBody, *base->rigidBody, t7, t8);
	dynamicsWorld->addConstraint(shellLatch);

	return base;
}

shared_ptr<PhysicsController> Turtle::CreateAndAttachLeg(glm::vec3 position, double legWidth, double legLength, double xScalar, double yScalar, double zScalar, shared_ptr<PhysicsController> base)
{
	//create leg
	shared_ptr<PhysicsController> leg = physicsFactory->CreateCapsule(legWidth, legLength, position, glm::quat());

	//connect leg to body
	btTransform t3, t4;
	t3.setIdentity(); 
	t4.setIdentity();
	t3.setOrigin(btVector3(xScalar, yScalar, zScalar));
	t4.setOrigin(btVector3(0, legLength, 0));
	btConeTwistConstraint * bodyUpperLegConeJoint = new btConeTwistConstraint(*leg->rigidBody, *base->rigidBody, t4, t3);
	bodyUpperLegConeJoint->setLimit(M_PI * 2, M_PI * 2, 0);
	dynamicsWorld->addConstraint(bodyUpperLegConeJoint);
	
	return leg;
}

void Turtle::FlipShell()
{
	//remove latch so shell can open
	dynamicsWorld->removeConstraint(shellLatch);
	shell->rigidBody->applyForce(btVector3(0, 20000, 0), btVector3(0, -(shellRadius / 4), shellRadius));
}