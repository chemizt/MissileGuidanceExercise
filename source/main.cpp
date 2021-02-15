#define _USE_MATH_DEFINES

#include <map>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <charconv>

#include "PIDController.hpp"
#include "utils.hpp"

using std::cout;
using std::string;
using std::map;
using std::ofstream;
using std::stod;
using std::fixed;
using std::setprecision;
using std::locale;
using std::ios_base;
using std::vector;
using std::stringstream;
using std::to_string;

#pragma region SIM_PARAMETERS
#define TARGET_ACCELERATION_DELTA_MIN -9
#define TARGET_ACCELERATION_DELTA_MAX 9
#define TARGET_HIT_RADIUS 15
#define MISSILE_G_TOLERANCE 30
#define MISSILE_PAYLOAD_MASS 230
#define MISSILE_FUEL_MASS 60
#define MISSILE_FUEL_ISP 235
#define MISSILE_ENGINE_BURN_TIME 6
#define POLAR_CURVE_BLADE_COEFFICIENT 1.5
#define GRAVITATIONAL_ACCELERATION 9.80665 
#define SIM_TIME_RESOLUTION_SECONDS 0.1

map<double, double> dragCoefficients {{0.5, 0.012}, {0.9, 0.015}, {1.2, 0.046}, {1.5, 0.044}, {2.0, 0.038}, {3.0, 0.030}, {4.0, 0.026}};
#pragma endregion

#define OUTPUT_FILE_NAME "outputData.csv"
#define STANDARD_PRECISION 5

vector<string> fileOutput;
vector<string> stdOutput;

class MovementVector
{
    public:
        MovementVector(double startX, double startY, double endX, double endY);
        double getStartX() { return _startX; }
        double getStartY() { return _startY; }
        double getEndX() { return _endX; }
        double getEndY() { return _endY; }
        double getXCoordinate() { return _endX - _startX; } // вычисляет координату X
        double getYCoordinate() { return _endY - _startY; } // вычисляет координату Y
        double getMagnitude() { return sqrt(pow(getXCoordinate(), 2) + pow(getYCoordinate(), 2)); } // вычисляет длину вектора по координатам
        void setStartX(double newStartX) { _startX = newStartX; }
        void setStartY(double newStartY) { _startY = newStartY; }
        void setEndX(double newEndX) { _endX = newEndX; }
        void setEndY(double newEndY) { _endY = newEndY; }
        void rotate(double angle); // поворачивает вектор на угол
        void normalize(); // нормализует вектор
    
    private:
        double _startX;
        double _startY;
        double _endX;
        double _endY;
};

class MovingObject // базовый класс движущихся объектов - имеет только скорость и направление движения
{
    public:
        MovingObject() = delete;
        MovingObject(double initialSpeed, double initialX, double initialY);
        ~MovingObject();
        double getX() { return _X; }
        double getY() { return _Y; }
        double getSpeed() { return _speed; }
        void setSpeed(double newSpeed) { _speed = newSpeed; }
        void basicMove();
    
    protected:
        void translateActingVectors(); // сдвигает действующие на объект векторы в соответствии с его новой позицией
        void rotateActingVectors(double angle); // поворачивает действующие на объект векторы в соответствии с углом, на который поворачивает объект
        double _X;
        double _Y;
        double _speed;
        map<string, MovementVector*> _actingVectors;
};

class Target : public MovingObject // класс целей - дополнительно имеет поперечное ускорение
{
    public:
        Target(double initialSpeed, double initialX, double initialY);
        double getAccelerationRate() { return _accelerationRate; }
        void setAccelerationRate(double newAccelerationRate) { _accelerationRate = newAccelerationRate; }
        void basicMove(double elapsedTime);
        void advancedMove();

    private:
        double _accelerationRate;
};

MovementVector::MovementVector(double startX, double startY, double endX, double endY)
{
    _startX = startX;
    _startY = startY;
    _endX = endX;
    _endY = endY;
}

void MovementVector::rotate(double angle)
{
    double rotatedX = getXCoordinate() * cos(angle) - getYCoordinate() * sin(angle);
    double rotatedY = getXCoordinate() * sin(angle) + getYCoordinate() * cos(angle);

    _endX = rotatedX;
    _endY = rotatedY;
}

void MovementVector::normalize() 
{
    double magnitude = this->getMagnitude();
    
    _endX = _endX / magnitude;
    _endY = _endY / magnitude;
}

MovingObject::MovingObject(double initialSpeed, double initialX, double initialY)
{
    MovementVector* velocity = new MovementVector(initialX, initialY, initialX, initialY + 1); // создаём вектор скорости, сонаправленный с осью Y
    _X = initialX;
    _Y = initialY;
    _actingVectors.insert({"velocity", velocity});
}

MovingObject::~MovingObject()
{
    for (const auto& [key, value] : _actingVectors)
    {
        delete value;
    };
}

void MovingObject::basicMove()
{
    MovementVector* velocity = _actingVectors.at("velocity");
    
    // изменяем положение объекта в соответствии с направлением вектора скорости и его величиной
    _X += velocity->getEndX() * _speed;
    _Y += velocity->getEndY() * _speed;
    
    // сдвигаем действующие векторы
    translateActingVectors();

    // поворачиваем действующие векторы (в данном случае только вектор скорости) на угол 30 градусов
    rotateActingVectors(convertDegreesToRadians(30));
}

void MovingObject::translateActingVectors()
{
    for (const auto& [key, vector] : _actingVectors)
    {
        vector->setStartX(_X); 
        vector->setStartY(_Y);
        vector->setEndX(vector->getEndX() + _X);
        vector->setEndY(vector->getEndY() + _Y);
    };
}

void MovingObject::rotateActingVectors(double angle)
{
    for (const auto& [key, vector] : _actingVectors)
    {
        vector->rotate(angle);
    };
}

Target::Target(double initialSpeed, double initialX, double initialY)
: MovingObject(initialSpeed, initialX, initialY)
{
    MovementVector* acceleration = new MovementVector(initialX, initialY, initialX + 1, initialY); // создаём вектор поперечного ускорения
    _accelerationRate = 0;
    _actingVectors.insert({"acceleration", acceleration});
}

void Target::basicMove(double elapsedTime)
{
    MovementVector* velocity = _actingVectors.at("velocity");
    MovementVector* acceleration = _actingVectors.at("acceleration");
    MovementVector* directionChange = new MovementVector(_X, _Y, 0, 0);
    double speedPositionDelta = elapsedTime * _speed; // вычисляем изменение координат из-за скорости
    double accelerationPositionDelta = pow(elapsedTime, 2) * _accelerationRate * GRAVITATIONAL_ACCELERATION; // вычисляем изменение координат из-за ускорения
    double scalarProduct;
    double determinant;
    double angle;

    // изменяем координаты
    _X += velocity->getXCoordinate() * speedPositionDelta + acceleration->getXCoordinate() * accelerationPositionDelta; 
    _Y += velocity->getYCoordinate() * speedPositionDelta + acceleration->getYCoordinate() * accelerationPositionDelta;

    // строим вектор изменения направления
    directionChange->setEndX(_X); directionChange->setEndY(_Y);

    // находим скалярное произведение и определитель векторов скорости и изменения направления
    scalarProduct = velocity->getXCoordinate() * directionChange->getXCoordinate() + velocity->getYCoordinate() * directionChange ->getYCoordinate();
    determinant = velocity->getXCoordinate() * directionChange ->getYCoordinate() - velocity->getYCoordinate() * directionChange->getXCoordinate();

    // находим угол между векторами скорости и изменения направления
    angle = atan2(determinant, scalarProduct);

    rotateActingVectors(angle);
    translateActingVectors();
    delete directionChange;
}

void Target::advancedMove()
{
    double simElapsedTime = 0;
    double timeSinceAccelerationChange = 0;
    double timeToProceedWithAcceleration = getRandomInRange(SIM_TIME_RESOLUTION_SECONDS, SIM_TIME_RESOLUTION_SECONDS * 200);
    string xCoordStr;
    string yCoordStr;
    string elapsedTimeStr;
    int i = 0;

    while (i < 600)
    {
        xCoordStr = to_string(_X);
        yCoordStr = to_string(_Y);
        elapsedTimeStr = to_string(simElapsedTime);
        elapsedTimeStr.replace(elapsedTimeStr.find("."), 1, ",");

        fileOutput.push_back(elapsedTimeStr + ";" + to_string(_X).replace(xCoordStr.find("."), 1, ",") + ";" + to_string(_Y).replace(yCoordStr.find("."), 1, ",") + "\n");
        stdOutput.push_back(xCoordStr + ", " + yCoordStr + "\n");

        basicMove(SIM_TIME_RESOLUTION_SECONDS);

        timeSinceAccelerationChange += SIM_TIME_RESOLUTION_SECONDS;
        simElapsedTime += SIM_TIME_RESOLUTION_SECONDS;

        if (timeSinceAccelerationChange >= timeToProceedWithAcceleration)
        {
            timeSinceAccelerationChange = 0;
            setAccelerationRate(getRandomInRange(-9, 9));
        }
        i++;
    }
    
    xCoordStr = to_string(_X);
    yCoordStr = to_string(_Y);
    elapsedTimeStr = to_string(simElapsedTime);
    elapsedTimeStr.replace(elapsedTimeStr.find("."), 1, ",");

    fileOutput.push_back(elapsedTimeStr + ";" + to_string(_X).replace(xCoordStr.find("."), 1, ",") + ";" + to_string(_Y).replace(yCoordStr.find("."), 1, ",") + "\n\n\n");
    stdOutput.push_back(xCoordStr + ", " + yCoordStr + "\n\n");
}

int main(int argc, char const* argv[])
{
    Target flyer(0.0, 0.0, 0.0);
    ofstream outputFile;
    
    int fileOutputNeeded = findArgumentInList(argc, argv, OUTPUT_TO_FILE_KEY);
    int speedIndex = findArgumentInList(argc, argv, SPEED_KEY);
    int accelIndex = findArgumentInList(argc, argv, ACCELERATION_KEY);
    
    if (speedIndex != -1) flyer.setSpeed(stod(argv[speedIndex + 1]));
    else flyer.setSpeed(getRandomInRange(100, 500));

    if (accelIndex != -1) flyer.setAccelerationRate(stod(argv[accelIndex + 1]) * GRAVITATIONAL_ACCELERATION);
    else flyer.setAccelerationRate(getRandomInRange(-9, 9));

    if (fileOutputNeeded != -1) 
    {
        outputFile.open(OUTPUT_FILE_NAME, ios_base::app);
        outputFile.imbue(locale(locale::classic(), new Comma));
        outputFile << fixed << setprecision(STANDARD_PRECISION) << "Time;X;Y\n";
    }

    cout << fixed << setprecision(STANDARD_PRECISION);
    
    fileOutput.clear();
    stdOutput.clear();

    flyer.advancedMove();
        
    for (string str : stdOutput)
    {
        cout << str;
    }

    if (fileOutputNeeded != -1)
    {
        for (string str : fileOutput) outputFile << str;
        outputFile.close();
    }
    
    return 0;
}
