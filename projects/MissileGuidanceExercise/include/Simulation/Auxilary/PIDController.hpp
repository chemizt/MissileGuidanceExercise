#ifndef PID_HDR_IG
#define PID_HDR_IG

class PIDController
{
	public:
		PIDController(double dT, double minBoundary, double maxBoundary, double kP, double kI, double kD);
		double calculate(double targetValue, double currentValue);
		void setMinBoundary(double newMinBoundary) { _minBoundary = newMinBoundary; }
		void setMaxBoundary(double newMaxBoundary) { _maxBoundary = newMaxBoundary; }
	
	private:
		double _dT;
		double _minBoundary;
		double _maxBoundary;
		double _kP;
		double _kI;
		double _kD;
		double _previousDeviation;
		double _integral;
};

#endif
