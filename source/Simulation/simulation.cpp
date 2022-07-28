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
	string tgtXCoord;
	string tgtYCoord;
	string tgtSpeed;
	string mslXCoord;
	string mslYCoord;
	string mslSpeed;
	string elapsedTimeStr;

	tgtXCoord = convertDoubleToStringWithPrecision(_target->getX());
	tgtYCoord = convertDoubleToStringWithPrecision(_target->getY());
	tgtSpeed = convertDoubleToStringWithPrecision(_target->getSpeed());
	mslXCoord = convertDoubleToStringWithPrecision(_missile->getX());
	mslYCoord = convertDoubleToStringWithPrecision(_missile->getY());
	mslSpeed = convertDoubleToStringWithPrecision(_missile->getSpeed());
	elapsedTimeStr = convertDoubleToStringWithPrecision(_simElapsedTime);

	if (_fileOutputNeeded) _outputFile << elapsedTimeStr + ";" + tgtXCoord + ";" + tgtYCoord + ";" + tgtSpeed + ";" + mslXCoord + ";" + mslYCoord + ";" + mslSpeed + ";\n";

	_target->advancedMove(SIM_RESOLUTION);
	_missile->advancedMove(SIM_RESOLUTION);

	_simElapsedTime += SIM_RESOLUTION;
}

bool Simulation::mslSpeedMoreThanTgtSpeed()
{
	return _missile->getRemainingFuelMass() > 0 ? true : _missile->getSpeed() > _target->getSpeed();
}

bool Simulation::mslWithinTgtHitRadius()
{
	return _getMslTgtDistance() <= _missile->getProxyRadius();
}

double Simulation::_getMslTgtDistance()
{
	return sqrt(pow(_missile->getX() - _target->getX(), 2) + pow(_missile->getY() - _target->getY(), 2));
}
