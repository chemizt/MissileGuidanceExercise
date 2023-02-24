#ifndef SIMULATION_HDR_IG
#define SIMULATION_HDR_IG

#include <QDebug>
#include "Auxilary/utils.hpp"
#include "Simulation/SimObjects/Target.hpp"
#include "Simulation/SimObjects/Missile.hpp"

class Simulation
{
	public:
		Simulation();
		Simulation(QPointF targetLocation, double targetSpeed, QPointF missileLocation, double missileSpeed, bool fileOutputNeeded);
		~Simulation();
		// проверяет присутствие ракеты в зоне поражения цели
		bool mslWithinTgtHitRadius() { return _getMslTgtDistance() <= _missile->getProxyRadius(); };
		// проверяет соотношение скоростей ракеты и цели
		bool mslSpeedMoreThanTgtSpeed() { return _missile->getRemainingFuelMass() > 0 ? true : _missile->getSpeed() > _target->getSpeed() && _missile->getTarget(); };
		Missile* getMissile() { return _missile; };
		Target* getTarget() { return _target; };
		void iterate();
		void setFileOutputNeededTo(const bool newVal);
		const double getMslProxyRadius() { return _missile->getProxyRadius(); };
		void restoreSimState()
		{
			if (_missile) _missile->restore();
			if (_target) _target->restore();
			if (_target && _missile) _missile->setTarget(_target);
			_simElapsedTime = 0.;
		};

	private:
		bool _fileOutputNeeded{ false };
		double _getMslTgtDistance() { return (_target->getCoordinates() - _missile->getCoordinates()).length(); };
		double _simElapsedTime{ 0. };
		Missile* _missile{ nullptr };
		ofstream _outputFile;
		Target* _target{ nullptr };
		void _prepOutputFile();
};

#endif // SIMULATION_HDR_IG
