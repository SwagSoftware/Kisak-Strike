#ifndef ICONTROLLER_H
#define ICONTROLLER_H

class CPhysicsObject;

class IController {
	public:
		// Bullet tick, called post-simulation
		virtual void Tick(float deltaTime) = 0;
};

#endif // ICONTROLLER_H
