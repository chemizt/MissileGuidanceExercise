#include "Simulation/SimObjects/Missile.hpp"
#include "Simulation/Auxilary/PIDController.hpp"
#include "Simulation/Auxilary/utils.hpp"

Missile::Missile(double initialSpeed, double initialX, double initialY) : MovingObject(initialSpeed, initialX, initialY)
{
}

void Missile::basicMove(double elapsedTime, double angleOfAttack)
{
	// изменяем скорость за счёт тяги двигателя и сопротивления воздуха
	_actingVectors.at("velocity") += _actingVectors.at("velocity").normalized() * (elapsedTime * (_calculatePropulsionAccelerationRate() - _calculateDragDecelerationRate(angleOfAttack)));

	// изменяем состояние ракеты
	_coordinates += _actingVectors.at("velocity") * elapsedTime;

	// потребляем топливо
	_remainingFuelMass -= std::min(_fuelConsumptionRate * elapsedTime, _remainingFuelMass);

	_timeSinceBirth += SIM_RESOLUTION;
}

void Missile::advancedMove(double elapsedTime)
{
	double steeringAngle = 0;

	if (_acquiredTarget && _timeSinceBirth >= _leDesc.apDelay)
	{
		QVector2D velocity = _actingVectors.at("velocity");
		QVector2D trgLOSVec = _acquiredTarget->getCoordinates() - getCoordinates();
		double velLOSAngle = getAngleBetweenVectorsRad(velocity.normalized(), trgLOSVec.normalized());

		if (abs(velLOSAngle) > degToRad(_leDesc.seekerMaxOBA))
		{
			_acquiredTarget = nullptr;
			return;
		}

		auto angleLimit = degToRad(_leDesc.seekerMaxOBA);
		steeringAngle = std::max(-angleLimit, std::min(degToRad(_navConstant * velLOSAngle), angleLimit));
		_rotateActingVectorsRad(steeringAngle);
	}

	basicMove(elapsedTime, radToDeg(steeringAngle));
}

double Missile::_calculateDragDecelerationRate(double angleOfAttack)
{
	double zeroLiftDragCoefficient;
	double liftInducedDragCoefficient = _calculateLiftInducedDragCoefficient(angleOfAttack); // вычисляем КИС
	double machNumber = _calculateMachNumber(SPEED_OF_SOUND); // вычисляем число Маха
	double fullDragForce = 0;
	QString machNumberStr = QString::fromStdString(convertDoubleToStringWithPrecision(machNumber, 1, false));

	if (_leDesc.cXData.contains(machNumberStr))
	{
		zeroLiftDragCoefficient = _leDesc.cXData.value(machNumberStr);
	}
	else
	{
		zeroLiftDragCoefficient = _interpolateZeroLiftDragCoefficient(machNumber);
	}

	fullDragForce = _calculateDynPressure() * (zeroLiftDragCoefficient + liftInducedDragCoefficient); // вычисляем полную силу сопротивления воздуха

	return fullDragForce / _calculateTotalMass(); // вычисляем "торможение", вызванное силой сопротивления воздуха
}

void Missile::_setGuidanceBoundary()
{
	double maxNormalForce = _calculateTotalMass() * _leDesc.maxAcceleration * FREEFALL_ACC; // вычисляем максимальную поперечную силу "торможения"
	double inducedDragCoeffAtMaxDecel;
	double maxAoA;

	inducedDragCoeffAtMaxDecel = maxNormalForce / _calculateDynPressure(); // вычисляем КИС при максимальной перегрузке

	// вычисляем угол атаки при максимальной перегрузке и задаём в соответствии с ним пределы наведения
	maxAoA = _calculateAngleOfAttack(inducedDragCoeffAtMaxDecel);
	_guidanceComputer->setMinBoundary(-maxAoA);
	_guidanceComputer->setMaxBoundary(maxAoA);
}

double Missile::_interpolateZeroLiftDragCoefficient(double machNumber)
{
	double interpolatedCoefficient = 0.01;

	if (machNumber < 0.5)
	{
		interpolatedCoefficient = lerp(machNumber, 0, 0, 0.5, _leDesc.cXData.value("0.5"));
	}
	else if (machNumber < 0.9)
	{
		interpolatedCoefficient = lerp(machNumber, 0.5, _leDesc.cXData.value("0.5"), 0.9, _leDesc.cXData.value("0.9"));
	}
	else if (machNumber < 1.2)
	{
		interpolatedCoefficient = lerp(machNumber, 0.9, _leDesc.cXData.value("0.9"), 1.2, _leDesc.cXData.value("1.2"));
	}
	else if (machNumber < 1.5)
	{
		interpolatedCoefficient = lerp(machNumber, 1.2, _leDesc.cXData.value("1.2"), 1.5, _leDesc.cXData.value("1.5"));
	}
	else if (machNumber < 2.0)
	{
		interpolatedCoefficient = lerp(machNumber, 1.5, _leDesc.cXData.value("1.5"), 2.0, _leDesc.cXData.value("2.0"));
	}
	else if (machNumber < 3.0)
	{
		interpolatedCoefficient = lerp(machNumber, 2.0, _leDesc.cXData.value("2.0"), 3.0, _leDesc.cXData.value("3.0"));
	}
	else if (machNumber < 4.0)
	{
		interpolatedCoefficient = lerp(machNumber, 3.0, _leDesc.cXData.value("3.0"), 4.0, _leDesc.cXData.value("4.0"));
	}

	return interpolatedCoefficient;
}
