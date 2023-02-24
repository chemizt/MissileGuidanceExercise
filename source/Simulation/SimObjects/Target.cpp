#include "Simulation/CommonSimParams.hpp"
#include "Simulation/SimObjects/Target.hpp"
#include "Simulation/Auxilary/utils.hpp"

namespace TargetParameters
{
	constexpr std::pair<double, double> evManeuverTimeConstraints{ 0.5, 30. };	// мин/макс время следования с ускорением для цели
	constexpr std::pair<double, double> evManeuverAccelConstraints{ -9., 9. };	// мин/макс поперечное ускорение цели
};

Target::Target(double initialSpeed, double initialX, double initialY) : MovingObject(-initialSpeed, initialX, initialY)
{
	_actingVectors.try_emplace("acceleration", QVector2D(0, 0)); // создаём вектор ускорения

	_setUpAccelerationParameters();
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
		_setUpAccelerationParameters();
}

inline void Target::_setUpAccelerationParameters()
{
	using namespace TargetParameters;

	_timeSinceAccelerationChange = 0;
	_timeToProceedWithAcceleration = _getRandomInRange(evManeuverTimeConstraints.first, evManeuverTimeConstraints.second);	// задаём случайное время следования с новым ускорением
	setAccelerationRate(_getRandomInRange(evManeuverAccelConstraints.first, evManeuverAccelConstraints.second));			// меняем ускорение
}
