#include "Simulation/CommonSimParams.hpp"
#include "Simulation/SimObjects/Target.hpp"
#include "Simulation/Auxilary/utils.hpp"

namespace TargetParameters
{
	double maxEvasiveManeuverTime = 30;			// максимальное время следования с ускорением для цели
	double maxNormalAcceleration = 9;			// верхний порог ускорения цели
	double minNormalAcceleration = -9;			// нижний порог ускорения цели
};

Target::Target(double initialSpeed, double initialX, double initialY) : MovingObject(-initialSpeed, initialX, initialY)
{
	_actingVectors.try_emplace("acceleration", QVector2D(1, 0)); // создаём вектор ускорения

	_timeSinceAccelerationChange = 0;
	_timeToProceedWithAcceleration = _getRandomInRange(TargetParameters::maxEvasiveManeuverTime * 0.5, TargetParameters::maxEvasiveManeuverTime);
}

double Target::getAccelerationRate()
{
	return _actingVectors.at("acceleration").length();
}

void Target::basicMove(double elapsedTime)
{
	QVector2D positionDelta = _actingVectors.at("velocity") * elapsedTime;

	if (_actingVectors.at("acceleration").length())
		 positionDelta += _actingVectors.at("acceleration") * pow(elapsedTime, 2) * FREEFALL_ACC * 0.5;

	_rotateActingVectorsRad(getAngleBetweenVectorsRad(positionDelta.normalized(), _actingVectors.at("velocity").normalized()));

	_coordinates += positionDelta;
	timeSinceBirth += SIM_RESOLUTION;
}

void Target::setAccelerationRate(double newAccelerationRate)
{
	_actingVectors.at("acceleration").setX(newAccelerationRate);
}

void Target::advancedMove(double elapsedTime)
{
	basicMove(elapsedTime);

	_timeSinceAccelerationChange += elapsedTime;

	if (_timeSinceAccelerationChange >= _timeToProceedWithAcceleration)
	{
		_timeSinceAccelerationChange = 0;
		setAccelerationRate(_getRandomInRange(TargetParameters::minNormalAcceleration, TargetParameters::maxNormalAcceleration)); // меняем ускорение
		_timeToProceedWithAcceleration = _getRandomInRange(TargetParameters::maxEvasiveManeuverTime * 0.5, TargetParameters::maxEvasiveManeuverTime); // задаём случайное время следования с новым ускорением
	}
}
