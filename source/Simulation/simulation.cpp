#include "Simulation/simulation.hpp"
#include "Simulation/CommonSimParams.hpp"
#include "Simulation/SimObjects/Target.hpp"
#include "Simulation/SimObjects/Missile.hpp"

Simulation::Simulation(QPointF targetLocation, double targetSpeed, QPointF missileLocation, double missileSpeed, bool fileOutputNeeded)
{
	_simElapsedTime = 0;
	_fileOutputNeeded = fileOutputNeeded;
	_target = new Target(targetSpeed, targetLocation.x(), targetLocation.y());
	_missile = new Missile(missileSpeed, missileLocation.x(), missileLocation.y());

	if (_fileOutputNeeded)
	{
		_outputFile.open(OUT_FILE_NAME, ios_base::out | ios_base::trunc);
		_outputFile << fixed << setprecision(STANDARD_PRECISION) << "Time;Target X;Target Y;Target Speed (m/s);Missile X;Missile Y;Missile Speed (m/s);\n";
	}

	_missile->setTarget(_target);
}

Simulation::~Simulation()
{
	delete _target;
	delete _missile;
	if (_fileOutputNeeded) _outputFile.close();
}

void Simulation::iterate()
{
	if (_fileOutputNeeded)
	{
		_outputFile << convertDoubleToStringWithPrecision(_target->getX()) + ";"
			+ convertDoubleToStringWithPrecision(_target->getY()) + ";"
			+ convertDoubleToStringWithPrecision(_target->getSpeed()) + ";"
			+ convertDoubleToStringWithPrecision(_missile->getX()) + ";"
			+ convertDoubleToStringWithPrecision(_missile->getY()) + ";"
			+ convertDoubleToStringWithPrecision(_missile->getSpeed()) + ";"
			+ convertDoubleToStringWithPrecision(_simElapsedTime) + ";\n";
	}

	_target->advancedMove(SIM_RESOLUTION);
	_missile->advancedMove(SIM_RESOLUTION);

	_simElapsedTime += SIM_RESOLUTION;
}

bool Simulation::mslSpeedMoreThanTgtSpeed()
{
	return _missile->getRemainingFuelMass() > 0 ? true : _missile->getSpeed() > _target->getSpeed() && _missile->getTarget();
}

bool Simulation::mslWithinTgtHitRadius()
{
	return _getMslTgtDistance() <= _missile->getProxyRadius();
}

double Simulation::_getMslTgtDistance()
{
	return (_target->getCoordinates() - _missile->getCoordinates()).length();
}
