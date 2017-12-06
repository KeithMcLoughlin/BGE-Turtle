#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class Turtle :
		public Game
	{
	private:

	public:
		Turtle(void);
		~Turtle(void);
		bool Initialise();
		void Update(float deltaTime);
		void Cleanup();
		void AddBall();
		shared_ptr<PhysicsController> ball;
		shared_ptr<PhysicsController> turtle;
		shared_ptr<PhysicsController> CreateTurtle(glm::vec3 position, float scale = 5);
		shared_ptr<PhysicsController> Turtle::CreateAndAttachLeg(glm::vec3 position, double legWidth, double legLength, double xScalar, double yScalar, double zScalar, shared_ptr<PhysicsController> base);
		double shellRadius;
		double headScale;

		//turtle parts
		shared_ptr<PhysicsController> base;
		shared_ptr<PhysicsController> head;
		shared_ptr<PhysicsController> tail;
		shared_ptr<PhysicsController> shell;
		shared_ptr<PhysicsController> rightFrontLeg;
		shared_ptr<PhysicsController> leftFrontLeg;
		shared_ptr<PhysicsController> rightBackLeg;
		shared_ptr<PhysicsController> leftBackLeg;

		const float FlipInterval = 3.0f;
		float timeBeforeFlippingShell;
		void FlipShell();
		btFixedConstraint * shellLatch;

		const btQuaternion legRotation = btQuaternion(0, 0, glm::half_pi<float>());
		const btVector3 legTorque = btVector3(10, 0, 0);
	};
}
