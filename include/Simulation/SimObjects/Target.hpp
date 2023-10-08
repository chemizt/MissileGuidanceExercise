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
		void setAccelerationRate(const double newAccelerationRate);
		void setEvasiveActionState(const bool newState) { _isEvasiveActionRequired = newState; restore(); };
		virtual void restore()
		{
			auto& accVec = _actingVectors.at("acceleration");
			accVec.setX(0); accVec.setY(0);

			_setUpAccelerationParameters();
		};

	private:
		bool _isEvasiveActionRequired{ true };
		double _timeSinceAccelerationChange;	// время, прошедшее с момента изменения ускорения
		double _timeToProceedWithAcceleration;	// временной промежуток для следования с текущим ускорением
		void _setUpAccelerationParameters();	// задаёт параметры ускорения
};

#endif // TARGET_HDR_IG
