#include "Simulation/CommonSimParams.hpp"
#include "Simulation/SimObjects/Target.hpp"
#include "Simulation/Auxilary/utils.hpp"

namespace TargetParameters
{
	double maxEvasiveManeuverTime = 5;			// максимальное время следования с ускорением для цели
	double maxNormalAcceleration = 9;			// верхний порог ускорения цели
	double minNormalAcceleration = -9;			// нижний порог ускорения цели
};

Target::Target(double initialSpeed, double initialX, double initialY) : MovingObject(-initialSpeed, initialX, initialY)
{
	QVector2D acceleration = QVector2D(1, 0); // создаём вектор поперечного ускорения

	_timeSinceAccelerationChange = 0;
	_accelerationRate = 0;
	_timeToProceedWithAcceleration = _getRandomInRange(SIM_RESOLUTION, TargetParameters::maxEvasiveManeuverTime);

	_actingVectors.insert("acceleration", acceleration);
}

double Target::getAccelerationRate()
{
	return _accelerationRate;
}

void Target::basicMove(double elapsedTime)
{
	QVector2D acceleration = _actingVectors.value("acceleration") * pow(elapsedTime, 2) * _accelerationRate * FREEFALL_ACC;
	QVector2D velocity = _actingVectors.value("velocity") * elapsedTime;

	_coordinates += velocity + acceleration;
}

void Target::setAccelerationRate(double newAccelerationRate)
{
	_accelerationRate = newAccelerationRate;
}

void Target::advancedMove(double elapsedTime)
{
	basicMove(elapsedTime);

	_timeSinceAccelerationChange += elapsedTime;

	if (_timeSinceAccelerationChange >= _timeToProceedWithAcceleration)
	{
		_timeSinceAccelerationChange = 0;
		setAccelerationRate(_getRandomInRange(-9, 9)); // меняем ускорение
		_timeToProceedWithAcceleration = _getRandomInRange(SIM_RESOLUTION * 10, TargetParameters::maxEvasiveManeuverTime); // задаём случайное время следования с новым ускорением
	}
}
