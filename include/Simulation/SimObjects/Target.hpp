#ifndef TARGET_HDR_IG
#define TARGET_HDR_IG

#include "MovingObject.hpp"

class Target : public MovingObject // класс целей - дополнительно имеет поперечное ускорение
{
	public:
		Target(double initialSpeed, double initialX, double initialY);
		double getAccelerationRate();
		void advancedMove(double elapsedTime);
		void basicMove(double elapsedTime);
		void setAccelerationRate(double newAccelerationRate);

	private:
		double _accelerationRate;
		double _timeSinceAccelerationChange;	// время, прошедшее с момента изменения ускорения
		double _timeToProceedWithAcceleration;	// временной промежуток для следования с текущим ускорением
};

#endif // TARGET_HDR_IG
