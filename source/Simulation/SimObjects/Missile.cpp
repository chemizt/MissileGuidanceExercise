#include "Simulation/CommonSimParams.hpp"
#include "Simulation/SimObjects/Missile.hpp"
#include "Simulation/Auxilary/PIDController.hpp"
#include "Simulation/Auxilary/utils.hpp"

namespace MissileParameters
{
	double motorBurnTime				= 6;	// время работы двигателя
	double motorSpecImpulse				= 235;	// удельный импульс топлива
	double motorFuelMass				= 60;	// масса топлива
	double maxAcceleration				= 30;	// порог перегрузки ракеты
	double emptyMass					= 230;	// масса ракеты без топлива
	double planformArea					= 0.9;	// характеристическая площадь ракеты
	double DyPerDa						= 1.5;	// отвал поляры
	double proxyFuzeRadius				= 15;	// радиус поражения цели
	QMap<QString, double> cXData {{"0.5", 0.012}, {"0.9", 0.015}, {"1.2", 0.046}, {"1.5", 0.044}, {"2.0", 0.038}, {"3.0", 0.030}, {"4.0", 0.026}};
};

Missile::Missile(double initialSpeed, double initialX, double initialY) : MovingObject(initialSpeed, initialX, initialY)
{
	_acquiredTarget = nullptr;
	_guidanceComputer = new PIDController(SIM_RESOLUTION, -5, 5, 0.001, 0.007, 0.003);
	_remainingFuelMass = MissileParameters::motorFuelMass;
	_fuelConsumptionRate = MissileParameters::motorFuelMass / MissileParameters::motorBurnTime;
	_engineThrust = MissileParameters::motorSpecImpulse * _fuelConsumptionRate * FREEFALL_ACC;
}

double Missile::getRemainingFuelMass()
{
	return _remainingFuelMass;
}

double Missile::getProxyRadius()
{
	return MissileParameters::proxyFuzeRadius;
}

void Missile::basicMove(double elapsedTime, double angleOfAttack)
{
	// изменяем скорость за счёт тяги двигателя и сопротивления воздуха
	_velocity += _velocity.normalized() * (elapsedTime * (_calculatePropulsionAccelerationRate() - _calculateDragDecelerationRate(angleOfAttack)));

	QVector2D velocity = _actingVectors.value("velocity");

	// изменяем состояние ракеты
	_coordinates.setX(_coordinates.x() + velocity.x() * elapsedTime);
	_coordinates.setY(_coordinates.y() + velocity.y() * elapsedTime);

	// потребляем топливо
	if (_remainingFuelMass > 0) _remainingFuelMass -= _fuelConsumptionRate * elapsedTime;
}

void Missile::setTarget(MovingObject* newTarget) { _acquiredTarget = newTarget; }

void Missile::advancedMove(double elapsedTime)
{
	QVector2D velocity = _actingVectors.value("velocity");
	QVector2D lead{}; // вычисляем вектор упреждения
	double steeringAngle;
	double angleDiff;

	_setGuidanceBoundary(); // определяем пределы наведения
	angleDiff = getAngleBetweenVectors(velocity, lead); // находим угол между векторами скорости и упреждения
	steeringAngle = abs(angleDiff) < 60 ?_guidanceComputer->calculate(0, angleDiff) : 0; // если угол > 60 гр., ракета "теряет" цель

	basicMove(elapsedTime, steeringAngle);

	_rotateActingVectors(steeringAngle);
}

double Missile::_calculateDragDecelerationRate(double angleOfAttack)
{
	double zeroLiftDragCoefficient;
	double liftInducedDragCoefficient = _calculateLiftInducedDragCoefficient(angleOfAttack); // вычисляем КИС
	double machNumber = _calculateMachNumber(SPEED_OF_SOUND); // вычисляем число Маха
	double aerodynamicParameter = _calculateDynPressure(); // вычисляем "аэродинамический параметр"
	double fullDragForce = 0;
	QString machNumberStr = QString::fromStdString(convertDoubleToStringWithPrecision(machNumber, 1, false));

	if (MissileParameters::cXData.contains(machNumberStr))
	{
		zeroLiftDragCoefficient = MissileParameters::cXData.value(machNumberStr);
	}
	else
	{
		zeroLiftDragCoefficient = _interpolateZeroLiftDragCoefficient(machNumber);
	}

	fullDragForce = aerodynamicParameter * (zeroLiftDragCoefficient + liftInducedDragCoefficient); // вычисляем полную силу сопротивления воздуха

	return fullDragForce / _calculateTotalMass(); // вычисляем "торможение", вызванное силой сопротивления воздуха
}

void Missile::_setGuidanceBoundary()
{
	double zeroLiftDragCoefficient;
	double machNumber = _calculateMachNumber(SPEED_OF_SOUND); // вычисляем число Маха
	double maxDecelForce = _calculateTotalMass() * MissileParameters::maxAcceleration * FREEFALL_ACC; // вычисляем максимальную силу "торможения"
	double inducedDragCoeffAtMaxDecel;
	double maxAoA;
	QString machNumberStr = QString::fromStdString(convertDoubleToStringWithPrecision(machNumber, 1, false));

	if (MissileParameters::cXData.contains(machNumberStr))
	{
		zeroLiftDragCoefficient = MissileParameters::cXData.value(machNumberStr);
	}
	else
	{
		zeroLiftDragCoefficient = _interpolateZeroLiftDragCoefficient(machNumber);
	}

	inducedDragCoeffAtMaxDecel = maxDecelForce / _calculateDynPressure() - zeroLiftDragCoefficient; // вычисляем КИС при максимальной перегрузке

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
		interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 0, 0, 0.5, MissileParameters::cXData.value("0.5"));
	}
	else if (machNumber < 0.9)
	{
		interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 0.5, MissileParameters::cXData.value("0.5"), 0.9, MissileParameters::cXData.value("0.9"));
	}
	else if (machNumber < 1.2)
	{
		interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 0.9, MissileParameters::cXData.value("0.9"), 1.2, MissileParameters::cXData.value("1.2"));
	}
	else if (machNumber < 1.5)
	{
		interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 1.2, MissileParameters::cXData.value("1.2"), 1.5, MissileParameters::cXData.value("1.5"));
	}
	else if (machNumber < 2.0)
	{
		interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 1.5, MissileParameters::cXData.value("1.5"), 2.0, MissileParameters::cXData.value("2.0"));
	}
	else if (machNumber < 3.0)
	{
		interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 2.0, MissileParameters::cXData.value("2.0"), 3.0, MissileParameters::cXData.value("3.0"));
	}
	else if (machNumber < 4.0)
	{
		interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 3.0, MissileParameters::cXData.value("3.0"), 4.0, MissileParameters::cXData.value("4.0"));
	}

	return interpolatedCoefficient;
}

double Missile::_calculateLiftInducedDragCoefficient(double dAoA)
{
	return dAoA * MissileParameters::DyPerDa;
}

double Missile::_calculateMachNumber(double c)
{
	return getSpeed() / c;
}

double Missile::_calculatePropulsionAccelerationRate()
{
	return _remainingFuelMass > 0 ? _engineThrust / _calculateTotalMass() : 0;
}

double Missile::_calculateTotalMass()
{
	return _remainingFuelMass + MissileParameters::emptyMass;
}

double Missile::_calculateAngleOfAttack(double inducedDragCoeff)
{
	return inducedDragCoeff / MissileParameters::DyPerDa;
}

double Missile::_calculateDynPressure()
{
	return (AIR_DENSITY * pow(getSpeed(), 2) * MissileParameters::planformArea) / 2;
}
