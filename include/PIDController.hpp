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

PIDController::PIDController(double dT, double minBoundary, double maxBoundary, double kP, double kI, double kD) : 
    _dT(dT),
    _minBoundary(minBoundary),
    _maxBoundary(maxBoundary),
    _kP(kP),
    _kI(kI),
    _kD(kD),
    _previousDeviation(0),
    _integral(0)
{

}

double PIDController::calculate(double targetValue, double currentValue)
{
    double currentDeviation = targetValue - currentValue; // вычисляем отклонение
    double outP = _kP * currentDeviation; // вычисляем пропорциональную составляющую

    // вычисляем интегральную составляющую
    currentDeviation * _previousDeviation > 0 ? _integral += currentDeviation * _dT : _integral = 0; // проверка смены знака отклонения или его обнуления
    double outI = _kI * _integral;                                                    // для устранения интегрального насыщения путём обнуления 
                                                                                       // интеграла // NOTE возможно, не нужно
    // вычисляем дифференциальную составляющую
    double derivative = (currentDeviation - _previousDeviation) / _dT;
    double outD = _kP * derivative;

    // вычисляем суммарную коррекцию
    double output = outP + outI + outD;

    // ограничиваем коррекцию в соответствии с пределами
    if (output < _minBoundary) output = _minBoundary;
    else if (output > _maxBoundary) output = _maxBoundary;

    // сохраняем текущее отклонение
    _previousDeviation = currentDeviation;

    return output;
}

#endif