#ifndef MISSILE_HDR_IG
#define MISSILE_HDR_IG

#include "MovingObject.hpp"

class PIDController;

class Missile : public MovingObject // класс ракет
{
	public:
		Missile(double initialSpeed, double initialX, double initialY);
		double getRemainingFuelMass();
		double getProxyRadius();
		void advancedMove(double elapsedTime);
		void basicMove(double elapsedTime, double angleOfAttack);
		void setTarget(MovingObject* newTarget);

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
