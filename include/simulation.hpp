#ifndef SIMULATION_HDR_IG
#define SIMULATION_HDR_IG

#include <QDebug>
#include <QMap>
#include <QPointF>
#include <QTransform>
#include <QVector2D>

#include "utils.hpp"
#include "PIDController.hpp"

#define AIR_DENSITY 1.293                       // плотность воздуха
#define GRAVITATIONAL_ACCELERATION 9.80665      // ускорение свободного падения
#define MAX_TIME_TO_PROCEED_WITH_ACCELERATION 5 // максимальное время следования с ускорением для цели
#define MISSILE_ENGINE_BURN_TIME 6              // время работы двигателя
#define MISSILE_FUEL_ISP 235                    // удельный импульс топлива
#define MISSILE_FUEL_MASS 60                    // масса топлива
#define MISSILE_G_TOLERANCE 30                  // порог перегрузки ракеты
#define MISSILE_PAYLOAD_MASS 230                // масса ракеты без топлива
#define PLANFORM_AREA 0.9                       // характерная площадь ракеты
#define POLAR_CURVE_BLADE_COEFFICIENT 1.5       // отвал поляры
#define SIM_TIME_RESOLUTION_SECONDS 0.01        // разрешение симуляции
#define SPEED_OF_SOUND 343                      // скорость звука
#define TARGET_ACCELERATION_DELTA_MAX 9         // верхний порог ускорения цели
#define TARGET_ACCELERATION_DELTA_MIN -9        // нижний порог ускорения цели
#define TARGET_HIT_RADIUS 15                    // радиус поражения цели
#define OUTPUT_FILE_NAME "outputData.csv"

class MovingObject // базовый класс движущихся объектов - имеет только скорость и направление движения
{
    public:
        MovingObject() = delete;
        MovingObject(double initialSpeed, double initialX, double initialY);
        ~MovingObject();
        double getSpeed() { return _speed; }
        double getX() { return _coordinates->x(); }
        double getY() { return _coordinates->y(); }
        QPointF* getCoordinates() { return _coordinates;}
        void setSpeed(double newSpeed) { _speed = newSpeed; }

    protected:
        double _speed;
        QMap<QString, QVector2D*> _actingVectors;
        QPointF* _coordinates;
        void _rotateActingVectors(double angle); // поворачивает действующие на объект векторы в соответствии с углом, на который поворачивает объект
};

class Target : public MovingObject // класс целей - дополнительно имеет поперечное ускорение
{
    public:
        Target(double initialSpeed, double initialX, double initialY);
        double getAccelerationRate() { return _accelerationRate; }
        QVector2D* getAcceleration() { return _actingVectors.value("acceleration"); }
        QVector2D* getVelocity() { return _actingVectors.value("velocity"); }
        void advancedMove(double elapsedTime);
        void basicMove(double elapsedTime);
        void setAccelerationRate(double newAccelerationRate) { _accelerationRate = newAccelerationRate; }

    private:
        double _accelerationRate;
        double _timeSinceAccelerationChange; // время, прошедшее с момента изменения ускорения
        double _timeToProceedWithAcceleration; // временной промежуток для следования с текущим ускорением
};

class Missile : public MovingObject // класс ракет
{
    public:
        Missile(double initialSpeed, double initialX, double initialY);
        double getRemainingFuelMass() { return _remainingFuelMass;}
        void advancedMove(double elapsedTime);
        void basicMove(double elapsedTime, double angleOfAttack);
        void setTarget(Target* newTarget) { _acquiredTarget = newTarget; }

    private:
        double _engineThrust;
        double _fuelConsumptionRate;
        double _remainingFuelMass;
        PIDController* _guidanceComputer;
        Target* _acquiredTarget;
        double _calculateAerodynamicParameter();                                                                                      // вычисляет "аэродинамический параметр" - 0.5 * rho * v ^ 2 * S
        double _calculateAngleOfAttack(double inducedDragCoeff);                                                                      // вычисляет угол атаки по коэфф. индуктивного сопротивления
        double _calculateDragDecelerationRate(double angleOfAttack);                                                                  // вычисляет "замедление", вызванное сопротивлением воздуха
        double _calculateLiftInducedDragCoefficient(double angleOfAttack);                                                            // вычисляет коэфф. индуктивного сопротивления по углу атаки
        double _calculateMachNumber() { return _speed / SPEED_OF_SOUND; };                                                            // вычисляет число Маха
        double _calculatePropulsionAccelerationRate() { return _remainingFuelMass > 0 ? _engineThrust / _calculateTotalMass() : 0; }  // вычисляет ускорение, вызванное тягой двигателя
        double _calculateTotalMass() { return _remainingFuelMass + MISSILE_PAYLOAD_MASS; }                                            // вычисляет полную массу ракеты
        double _interpolateZeroLiftDragCoefficient(double machNumber);                                                                // вычисляет коэфф. сопротивления формы по числу Маха
        QVector2D _calculateLead();                                                                                                   // вычисляет вектор упреждения
        void _setGuidanceBoundary();                                                                                                  // задаёт пределы углов наведения, выдаваемых регулятором наведения
};

class Simulation
{
    public:
        Simulation(QPointF targetLocation, double targetSpeed, QPointF missileLocation, double missileSpeed, bool fileOutputNeeded);
        ~Simulation();
        bool mslNotWithinTgtHitRadius(); // проверяет присутствие ракеты в зоне поражения цели
        bool mslSpeedMoreThanTgtSpeed(); // проверяет соотношение скоростей ракеты и цели
        Missile* getMissile() { return _missile; };
        Target* getTarget() { return _target; };
        void iterate();

    private:
        bool _fileOutputNeeded;
        double _getMslTgtDistance();
        double _simElapsedTime;
        Missile* _missile;
        ofstream _outputFile;
        Target* _target;
};

#endif // SIMULATION_HDR_IG
