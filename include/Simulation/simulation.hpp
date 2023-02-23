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
		bool mslWithinTgtHitRadius();				// проверяет присутствие ракеты в зоне поражения цели
		bool mslSpeedMoreThanTgtSpeed();			// проверяет соотношение скоростей ракеты и цели
		Missile* getMissile() { return _missile; };
		Target* getTarget() { return _target; };
		void iterate();
		void setFileOutputNeededTo(const bool newVal);
		static double getMslProxyRadius() { return MissileParameters::proxyFuzeRadius; };

	private:
		bool _fileOutputNeeded{ false };
		double _getMslTgtDistance();
		double _simElapsedTime{ 0. };
		Missile* _missile{ nullptr };
		ofstream _outputFile;
		Target* _target{ nullptr };
		void _prepOutputFile();
};

#endif // SIMULATION_HDR_IG
