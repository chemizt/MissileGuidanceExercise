#ifndef SIMULATION_HDR_IG
#define SIMULATION_HDR_IG

#include <QPointF>
#include <QMap>
#include <QVector2D>
#include <QTransform>
#include <QDebug>

#include "utils.hpp"
#include "PIDController.hpp"

#define TARGET_ACCELERATION_DELTA_MIN -9        // нижний порог ускорения цели
#define TARGET_ACCELERATION_DELTA_MAX 9         // верхний порог ускорения цели
#define TARGET_HIT_RADIUS 15                    // радиус поражения цели
#define MISSILE_G_TOLERANCE 30                  // порог перегрузки ракеты
#define MISSILE_PAYLOAD_MASS 230                // масса ракеты без топлива
#define MISSILE_FUEL_MASS 60                    // масса топлива
#define MISSILE_FUEL_ISP 235                    // удельный импульс топлива
#define MISSILE_ENGINE_BURN_TIME 6              // время работы двигателя
#define POLAR_CURVE_BLADE_COEFFICIENT 1.5       // отвал поляры
#define PLANFORM_AREA 0.9                       // характерная площадь ракеты
#define GRAVITATIONAL_ACCELERATION 9.80665      // ускорение свободного падения
#define SIM_TIME_RESOLUTION_SECONDS 0.01        // разрешение симуляции
#define MAX_TIME_TO_PROCEED_WITH_ACCELERATION 5 // максимальное время следования с ускорением для цели
#define SPEED_OF_SOUND 343                      // скорость звука
#define AIR_DENSITY 1.293                       // плотность воздуха
#define OUTPUT_FILE_NAME "outputData.csv"

class MovingObject // базовый класс движущихся объектов - имеет только скорость и направление движения
{
    public:
        MovingObject() = delete;
        MovingObject(double initialSpeed, double initialX, double initialY);
        ~MovingObject();
        double getX() { return _coordinates->x(); }
        double getY() { return _coordinates->y(); }
        double getSpeed() { return _speed; }
        QPointF* getCoordinates() { return _coordinates;}
        void setSpeed(double newSpeed) { _speed = newSpeed; }

    protected:
        void _rotateActingVectors(double angle); // поворачивает действующие на объект векторы в соответствии с углом, на который поворачивает объект
        QPointF* _coordinates;
        double _speed;
        QMap<QString, QVector2D*> _actingVectors;
};

class Target : public MovingObject // класс целей - дополнительно имеет поперечное ускорение
{
    public:
        Target(double initialSpeed, double initialX, double initialY);
        double getAccelerationRate() { return _accelerationRate; }
        QVector2D* getVelocity() { return _actingVectors.value("velocity"); }
        QVector2D* getAcceleration() { return _actingVectors.value("acceleration"); }
        void setAccelerationRate(double newAccelerationRate) { _accelerationRate = newAccelerationRate; }
        void basicMove(double elapsedTime);
        void advancedMove(double elapsedTime);

    private:
        double _accelerationRate;
        double _timeSinceAccelerationChange; // время, прошедшее с момента изменения ускорения
        double _timeToProceedWithAcceleration; // временной промежуток для следования с текущим ускорением
};

class Missile : public MovingObject // класс ракет
{
    public:
        Missile(double initialSpeed, double initialX, double initialY);
        void setTarget(Target* newTarget) { _acquiredTarget = newTarget; }
        double getRemainingFuelMass() { return _remainingFuelMass;}
        void basicMove(double elapsedTime, double angleOfAttack);
        void advancedMove(double elapsedTime);

    private:
        Target* _acquiredTarget;
        PIDController* _guidanceComputer;
        double _remainingFuelMass;
        double _fuelConsumptionRate;
        double _engineThrust;
        double _calculateTotalMass() { return _remainingFuelMass + MISSILE_PAYLOAD_MASS; }
        double _calculatePropulsionAccelerationRate() { return _remainingFuelMass > 0 ? _engineThrust / _calculateTotalMass() : 0; }
        double _calculateDragDecelerationRate(double angleOfAttack);
        void _setGuidanceBoundary();
        double _calculateMachNumber() { return _speed / SPEED_OF_SOUND; };
        double _interpolateZeroLiftDragCoefficient(double machNumber);
        double _calculateLiftInducedDragCoefficient(double angleOfAttack);
        double _calculateAerodynamicParameter();
        QVector2D _calculateLead();
        double _calculateAngleOfAttack(double inducedDragCoeff);
};

class Simulation
{
    public:
        Simulation(QPointF targetLocation, double targetSpeed, QPointF missileLocation, double missileSpeed, bool fileOutputNeeded);
        ~Simulation();
        void iterate();
        bool mslSpeedMoreThanTgtSpeed();
        bool mslNotWithinTgtHitRadius();
        Target* getTarget() { return _target; };
        Missile* getMissile() { return _missile; };

    private:
        double _simElapsedTime;
        bool _fileOutputNeeded;
        Target* _target;
        Missile* _missile;
        ofstream _outputFile;
        double _getMslTgtDistance();
};

#endif // SIMULATION_HDR_IG
