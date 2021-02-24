#include "simulation.hpp"

QMap<QString, double> dragCoefficients {{"0.5", 0.012}, {"0.9", 0.015}, {"1.2", 0.046}, {"1.5", 0.044}, {"2.0", 0.038}, {"3.0", 0.030}, {"4.0", 0.026}};

MovingObject::MovingObject(double initialSpeed, double initialX, double initialY)
{
    _coordinates = new QPointF(initialX, initialY);
    QVector2D* velocity = new QVector2D(0, 1); // создаём вектор скорости, сонаправленный с осью Y

    _speed = initialSpeed;

    _actingVectors.insert("velocity", velocity);
}

MovingObject::~MovingObject()
{
    foreach(QVector2D* value, _actingVectors) delete value;
    delete _coordinates;
}

void MovingObject::_rotateActingVectors(double angle)
{
    foreach(QVector2D* value, _actingVectors)
    {
        QTransform transform = QTransform().rotate(angle);
        QPointF rotatedPoint = transform.map(value->toPointF());
        
        value->setX(rotatedPoint.x());
        value->setY(rotatedPoint.y());
    }
}

Target::Target(double initialSpeed, double initialX, double initialY) : MovingObject(initialSpeed, initialX, initialY)
{
    QVector2D* acceleration = new QVector2D(1, 0); // создаём вектор поперечного ускорения

    _timeSinceAccelerationChange = 0;
    _accelerationRate = 0;
    _timeToProceedWithAcceleration = getRandomInRange(SIM_TIME_RESOLUTION_SECONDS, MAX_TIME_TO_PROCEED_WITH_ACCELERATION);

    _actingVectors.insert("acceleration", acceleration);
    _actingVectors.value("velocity")->setY(-1);
}

void Target::basicMove(double elapsedTime)
{
    double accelerationPositionDelta = pow(elapsedTime, 2) * _accelerationRate * GRAVITATIONAL_ACCELERATION; // вычисляем изменение координат из-за ускорения
    double angle;
    double speedPositionDelta = elapsedTime * _speed; // вычисляем изменение координат из-за скорости
    QVector2D* acceleration = _actingVectors.value("acceleration");
    QVector2D* directionChange = new QVector2D(_coordinates->x(), _coordinates->y());
    QVector2D* velocity = _actingVectors.value("velocity");

    // изменяем координаты
    _coordinates->setX(_coordinates->x() + velocity->x() * speedPositionDelta + acceleration->x() * accelerationPositionDelta);
    _coordinates->setY(_coordinates->y() + velocity->y() * speedPositionDelta + acceleration->y() * accelerationPositionDelta);

    // строим вектор изменения направления
    directionChange->setX(velocity->x() * speedPositionDelta + acceleration->x() * accelerationPositionDelta);
    directionChange->setY(velocity->y() * speedPositionDelta + acceleration->y() * accelerationPositionDelta);
    directionChange->normalize();

    // находим угол между векторами скорости и изменения направления
    angle = getAngleBetweenVectors(*velocity, *directionChange);

    _rotateActingVectors(angle);
    delete directionChange;
}

void Target::advancedMove(double elapsedTime)
{
    basicMove(elapsedTime);

    _timeSinceAccelerationChange += elapsedTime;

    if (_timeSinceAccelerationChange >= _timeToProceedWithAcceleration)
    {
        _timeSinceAccelerationChange = 0;
        setAccelerationRate(getRandomInRange(-9, 9)); // меняем ускорение
        _timeToProceedWithAcceleration = getRandomInRange(SIM_TIME_RESOLUTION_SECONDS * 10, MAX_TIME_TO_PROCEED_WITH_ACCELERATION); // задаём случайное время следования с новым ускорением
    }
}

Missile::Missile(double initialSpeed, double initialX, double initialY) : MovingObject(initialSpeed, initialX, initialY)
{
    _acquiredTarget = NULL;
    _guidanceComputer = new PIDController(SIM_TIME_RESOLUTION_SECONDS, -5, 5, 0.001, 0.007, 0.003);
    _remainingFuelMass = MISSILE_FUEL_MASS;
    _fuelConsumptionRate = MISSILE_FUEL_MASS / MISSILE_ENGINE_BURN_TIME;
    _engineThrust = MISSILE_FUEL_ISP * _fuelConsumptionRate * GRAVITATIONAL_ACCELERATION;
}

void Missile::basicMove(double elapsedTime, double angleOfAttack)
{
    // изменяем скорость за счёт тяги двигателя и сопротивления воздуха
    _speed += elapsedTime * (_calculatePropulsionAccelerationRate() - _calculateDragDecelerationRate(angleOfAttack));
    // вычисляем изменение координат из-за скорости
    double speedPositionDelta = elapsedTime * _speed; 

    QVector2D* velocity = _actingVectors.value("velocity");

    // изменяем состояние ракеты
    _coordinates->setX(_coordinates->x() + velocity->x() * speedPositionDelta);
    _coordinates->setY(_coordinates->y() + velocity->y() * speedPositionDelta);

    // потребляем топливо
    if (_remainingFuelMass > 0) _remainingFuelMass -= _fuelConsumptionRate * elapsedTime;
}

void Missile::advancedMove(double elapsedTime)
{
    QVector2D* velocity = _actingVectors.value("velocity");
    QVector2D lead = _calculateLead(); // вычисляем вектор упреждения
    double steeringAngle;
    double angleDiff;

    _setGuidanceBoundary(); // определяем пределы наведения
    angleDiff = getAngleBetweenVectors(*velocity, lead); // находим угол между векторами скорости и упреждения
    steeringAngle = _guidanceComputer->calculate(abs(angleDiff) < 60 ? angleDiff : 0, 0); // если угол > 60 гр., ракета "теряет" цель

    basicMove(elapsedTime, steeringAngle);

    _rotateActingVectors(steeringAngle);
}

double Missile::_calculateDragDecelerationRate(double angleOfAttack)
{
    double zeroLiftDragCoefficient;
    double liftInducedDragCoefficient = _calculateLiftInducedDragCoefficient(angleOfAttack); // вычисляем КИС
    double machNumber = _calculateMachNumber(); // вычисляем число Маха
    double aerodynamicParameter = _calculateAerodynamicParameter(); // вычисляем "аэродинамический параметр"
    double fullDragForce = 0;
    QString machNumberStr = QString::fromStdString(convertDoubleToStringWithPrecision(machNumber, 1, false));

    if (dragCoefficients.contains(machNumberStr))
    {
        zeroLiftDragCoefficient = dragCoefficients.value(machNumberStr);
    }
    else
    {
        zeroLiftDragCoefficient = _interpolateZeroLiftDragCoefficient(machNumber);
    }

    fullDragForce = aerodynamicParameter * (zeroLiftDragCoefficient + liftInducedDragCoefficient); // вычисляем полную силу сопротивления воздуха

    return fullDragForce / _calculateTotalMass(); // вычисляем "торможение", вызванное силой сопротивления воздуха
}

void Missile::_setGuidanceBoundary()
{
    double zeroLiftDragCoefficient;
    double machNumber = _calculateMachNumber(); // вычисляем число Маха
    double maxDecelForce = _calculateTotalMass() * MISSILE_G_TOLERANCE * GRAVITATIONAL_ACCELERATION; // вычисляем максимальную силу "торможения"
    double inducedDragCoeffAtMaxDecel;
    double maxAoA;
    QString machNumberStr = QString::fromStdString(convertDoubleToStringWithPrecision(machNumber, 1, false));

    if (dragCoefficients.contains(machNumberStr))
    {
        zeroLiftDragCoefficient = dragCoefficients.value(machNumberStr);
    }
    else
    {
        zeroLiftDragCoefficient = _interpolateZeroLiftDragCoefficient(machNumber);
    }

    inducedDragCoeffAtMaxDecel = maxDecelForce / _calculateAerodynamicParameter() - zeroLiftDragCoefficient; // вычисляем КИС при максимальной перегрузке

     // вычисляем угол атаки при максимальной перегрузке и задаём в соответствии с ним пределы наведения
    maxAoA = _calculateAngleOfAttack(inducedDragCoeffAtMaxDecel);
    _guidanceComputer->setMinBoundary(-maxAoA);
    _guidanceComputer->setMaxBoundary(maxAoA);
}

double Missile::_interpolateZeroLiftDragCoefficient(double machNumber)
{
    double interpolatedCoefficient = 0.01;

    if (machNumber < 0.5)
    {
        interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 0, 0, 0.5, dragCoefficients.value("0.5"));
    }
    else if (machNumber < 0.9)
    {
        interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 0.5, dragCoefficients.value("0.5"), 0.9, dragCoefficients.value("0.9"));
    }
    else if (machNumber < 1.2)
    {
        interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 0.9, dragCoefficients.value("0.9"), 1.2, dragCoefficients.value("1.2"));
    }
    else if (machNumber < 1.5)
    {
        interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 1.2, dragCoefficients.value("1.2"), 1.5, dragCoefficients.value("1.5"));
    }
    else if (machNumber < 2.0)
    {
        interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 1.5, dragCoefficients.value("1.5"), 2.0, dragCoefficients.value("2.0"));
    }
    else if (machNumber < 3.0)
    {
        interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 2.0, dragCoefficients.value("2.0"), 3.0, dragCoefficients.value("3.0"));
    }
    else if (machNumber < 4.0)
    {
        interpolatedCoefficient = interpolateWithLinearInterpolation(machNumber, 3.0, dragCoefficients.value("3.0"), 4.0, dragCoefficients.value("4.0"));
    }

    return interpolatedCoefficient;
}

double Missile::_calculateLiftInducedDragCoefficient(double angleOfAttack)
{
    // Ci = A * (1/A) ^ 2 * alpha ^ 2
    return pow(angleOfAttack / POLAR_CURVE_BLADE_COEFFICIENT, 2) * POLAR_CURVE_BLADE_COEFFICIENT;
}

double Missile::_calculateAngleOfAttack(double inducedDragCoeff)
{
    // Ci = A * (1/A) ^ 2 * alpha ^ 2 => alpha ^ 2  = Ci / A * (1/A) ^ 2 => alpha = sqrt(Ci / A * (1/A) ^ 2)
    return sqrt(inducedDragCoeff / POLAR_CURVE_BLADE_COEFFICIENT * pow(1 / POLAR_CURVE_BLADE_COEFFICIENT, 2));
}

double Missile::_calculateAerodynamicParameter()
{
    // 0.5 * rho * v ^ 2 * S
    return (AIR_DENSITY * pow(_speed, 2) * PLANFORM_AREA) / 2;
}

QVector2D Missile::_calculateLead()
{
    double tgtSpeed = _acquiredTarget->getSpeed();
    QVector2D* targetVelocity = _acquiredTarget->getVelocity();
    QPointF predictedTargetLocation(_acquiredTarget->getX() + targetVelocity->x() * tgtSpeed, _acquiredTarget->getY() + targetVelocity->y() * tgtSpeed); // "предсказываем" положение цели
    QVector2D interceptVector(predictedTargetLocation.x() - _coordinates->x(), predictedTargetLocation.y() - _coordinates->y()); // строим вектор, указывающий на предсказанное положение цели

    return interceptVector.normalized();
}

Simulation::Simulation(QPointF targetLocation, double targetSpeed, QPointF missileLocation, double missileSpeed, bool fileOutputNeeded)
{
    _simElapsedTime = 0;
    _fileOutputNeeded = fileOutputNeeded;
    _target = new Target(targetSpeed, targetLocation.x(), targetLocation.y());
    _missile = new Missile(missileSpeed, missileLocation.x(), missileLocation.y());

    if (_fileOutputNeeded)
    {
        _outputFile.open(OUTPUT_FILE_NAME, ios_base::out | ios_base::trunc);
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

    _target->advancedMove(SIM_TIME_RESOLUTION_SECONDS);
    _missile->advancedMove(SIM_TIME_RESOLUTION_SECONDS);

    _simElapsedTime += SIM_TIME_RESOLUTION_SECONDS;
}

bool Simulation::mslSpeedMoreThanTgtSpeed()
{
    return _missile->getRemainingFuelMass() > 0 ? true : _missile->getSpeed() > _target->getSpeed();
}

bool Simulation::mslNotWithinTgtHitRadius()
{
    return _getMslTgtDistance() > TARGET_HIT_RADIUS;
}

double Simulation::_getMslTgtDistance()
{
    return sqrt(pow(_missile->getX() - _target->getX(), 2) + pow(_missile->getY() - _target->getY(), 2));
}
