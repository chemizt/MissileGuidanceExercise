#ifndef MISSILE_HDR_IG
#define MISSILE_HDR_IG

#include "MovingObject.hpp"

class PIDController;

namespace MissileParameters
{
	double motorBurnTime				= 6;	// время работы двигателя
	double motorSpecImpulse				= 235;	// удельный импульс топлива
	double motorFuelMass				= 60;	// масса топлива
	double maxAcceleration				= 30;	// порог перегрузки ракеты
	double emptyMass					= 230;	// масса ракеты без топлива
	double planformArea					= 0.9;	// характеристическая площадь ракеты
	double DyPerDa						= 1.5;	// отвал поляры
	double proxyFuzeRadius				= 15;	// радиус поражения цели (срабатывания НВ)
	double seekerMaxOBA					= 15;	// ширина ПЗ ГСН в одну сторону
	double navConstant					= 1.5;	// постоянная наведения
	QMap<QString, double> cXData {{"0.5", 0.012}, {"0.9", 0.015}, {"1.2", 0.046}, {"1.5", 0.044}, {"2.0", 0.038}, {"3.0", 0.030}, {"4.0", 0.026}};
};

class Missile : public MovingObject // класс ракет
{
	public:
		Missile(double initialSpeed, double initialX, double initialY);
		double getRemainingFuelMass();
		double getProxyRadius();
		MovingObject* getTarget();
		void advancedMove(double elapsedTime);
		void basicMove(double elapsedTime, double angleOfAttack);
		void setTarget(MovingObject* newTarget);
		void setNavConstant(double mslNavConstant);

	private:
		double _engineThrust;
		double _fuelConsumptionRate;
		double _remainingFuelMass;
		PIDController* _guidanceComputer;
		MovingObject* _acquiredTarget;
		double _calculateDynPressure();										// вычисляет скоростной напор - 0.5 * rho * v ^ 2 * S
		double _calculateAngleOfAttack(double inducedDragCoeff);			// вычисляет угол атаки по коэфф. индуктивного сопротивления
		double _calculateDragDecelerationRate(double angleOfAttack);		// вычисляет "замедление", вызванное сопротивлением воздуха
		double _calculateLiftInducedDragCoefficient(double angleOfAttack);	// вычисляет коэфф. индуктивного сопротивления по углу атаки
		double _calculateMachNumber(double c);								// вычисляет число Маха
		double _calculatePropulsionAccelerationRate();						// вычисляет ускорение, вызванное тягой двигателя
		double _calculateTotalMass();										// вычисляет полную массу ракеты
		double _interpolateZeroLiftDragCoefficient(double machNumber);		// вычисляет коэфф. сопротивления формы по числу Маха
		void _setGuidanceBoundary();										// задаёт пределы углов наведения, выдаваемых регулятором наведения
};

#endif // MISSILE_HDR_IG
