#ifndef SIMULATION_HDR_IG
#define SIMULATION_HDR_IG

#include <QDebug>
#include "Auxilary/utils.hpp"

class Missile;
class Target;

class Simulation
{
	public:
		Simulation(QPointF targetLocation, double targetSpeed, QPointF missileLocation, double missileSpeed, bool fileOutputNeeded);
		~Simulation();
		bool mslWithinTgtHitRadius();	// проверяет присутствие ракеты в зоне поражения цели
		bool mslSpeedMoreThanTgtSpeed();	// проверяет соотношение скоростей ракеты и цели
		Missile* getMissile() { return _missile; };
		Target* getTarget() { return _target; };
		void iterate();
		static double getMslProxyRadius();

	private:
		bool _fileOutputNeeded;
		double _getMslTgtDistance();
		double _simElapsedTime;
		Missile* _missile;
		ofstream _outputFile;
		Target* _target;
};

#endif // SIMULATION_HDR_IG
