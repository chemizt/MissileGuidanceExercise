#include "Simulation/simulation.hpp"
#include "Simulation/CommonSimParams.hpp"

Simulation::Simulation()
{
	_target = new Target(0, 0, 1e5);
	_missile = new Missile(0, 0, 0);
	_missile->setTarget(_target);
}

Simulation::Simulation(QPointF targetLocation, double targetSpeed, QPointF missileLocation, double missileSpeed, bool fileOutputNeeded)
{
	_fileOutputNeeded = fileOutputNeeded;
	_target = new Target(targetSpeed, targetLocation.x(), targetLocation.y());
	_missile = new Missile(missileSpeed, missileLocation.x(), missileLocation.y());

	if (_fileOutputNeeded)
		_prepOutputFile();

	_missile->setTarget(_target);
}

Simulation::~Simulation()
{
	delete _target;
	delete _missile;
	if (_outputFile.is_open()) _outputFile.close();
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

void Simulation::setFileOutputNeededTo(const bool newVal)
{
	if (newVal && !_outputFile.is_open())
		_prepOutputFile();
	else if (_outputFile.is_open())
	{
		_outputFile << "FILE;HAS;;BEEN;;FORCIBLY;CLOSED;\n";
		_outputFile.close();
	}
}

void Simulation::_prepOutputFile()
{
	_outputFile.open(OUT_FILE_NAME, ios_base::out | ios_base::trunc);
	_outputFile << fixed << setprecision(STANDARD_PRECISION) << "Time;Target X;Target Y;Target Speed (m/s);Missile X;Missile Y;Missile Speed (m/s);\n";
}
