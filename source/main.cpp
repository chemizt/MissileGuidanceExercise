#define _USE_MATH_DEFINES

#include <map>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>

#include <PIDController.hpp>

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

std::map<double, double> dragCoefficients {{0.5, 0.012}, {0.9, 0.015}, {1.2, 0.046}, {1.5, 0.044}, {2.0, 0.038}, {3.0, 0.030}, {4.0, 0.026}};

#pragma endregion

#define OUTPUT_FILE_NAME "outputData.csv"

struct Comma final : std::numpunct<char> // inspired by (copy-typed from) https://stackoverflow.com/a/42331536
{
    char do_decimal_point() const override { return ','; }
};

int findArgumentInList(int argc, char const* argv[], std::string arg)
{
    int result = -1;
    
    for (unsigned int i = 0; (i < argc) && (result == -1); i++)
    {
        if (argv[i] == arg) result = i;
    }

    return result;
}

class MovementVector
{
    public:
        MovementVector(double startX, double startY, double endX, double endY);
        double getStartX() { return _startX; }
        double getStartY() { return _startY; }
        double getEndX() { return _endX; }
        double getEndY() { return _endY; }
        double getLength() { return sqrt(pow(_endX - _startX, 2) + pow(_endY - _startY, 2)); } // вычисляем длину вектора по координатам
        void setStartX(double newStartX) { _startX = newStartX; }
        void setStartY(double newStartY) { _startY = newStartY; }
        void setEndX(double newEndX) { _endX = newEndX; }
        void setEndY(double newEndY) { _endY = newEndY; }
        void rotate(double angle);
    
    private:
        double _startX;
        double _startY;
        double _endX;
        double _endY;
};

class MovingObject
{
    public:
        MovingObject() = delete;
        MovingObject(double initialSpeed, double initialX, double initialY);
        ~MovingObject();
        double getX() { return _X; }
        double getY() { return _Y; }
        double getSpeed() { return _speed; }
        void setSpeed(double newSpeed) { _speed = newSpeed; }
        void move();
    
    private:
        double _X;
        double _Y;
        double _speed;
        MovementVector* _velocity = NULL;
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
    double rotatedX = (_endX - _startX) * cos(angle) - (_endY - _startY) * sin(angle);
    double rotatedY = (_endX - _startX) * sin(angle) + (_endY - _startY) * cos(angle);

    _endX = rotatedX;
    _endY = rotatedY;
}

MovingObject::MovingObject(double initialSpeed, double initialX, double initialY)
{
    _X = initialX;
    _Y = initialY;
    _velocity = new MovementVector(initialX, initialY, 0, 1);
}

MovingObject::~MovingObject()
{
    delete _velocity;
}

void MovingObject::move()
{
    _X += _velocity->getEndX() * _speed;
    _Y += _velocity->getEndY() * _speed;
    _velocity->rotate(-20 * M_PI / 180.0);
}

int main(int argc, char const* argv[])
{
    MovingObject flyer(0.0, 0.0, 0.0);
    bool needToContinue;
    std::ofstream outputFile;

    int speedIndex = findArgumentInList(argc, argv, "-s");
    if (speedIndex != -1) flyer.setSpeed(std::stod(argv[speedIndex + 1]));
    else flyer.setSpeed(10.0);

    outputFile.open(OUTPUT_FILE_NAME, std::ios_base::app);
    outputFile.imbue(std::locale(std::locale::classic(), new Comma));
    outputFile << std::fixed << std::setprecision(2) << "X" << ";" << "Y" << "\n";

    do
    {
        std::cout << std::fixed << std::setprecision(2);
        
        if (findArgumentInList(argc, argv, "-p") != -1) std::cout << flyer.getX() << ", " << flyer.getY() << "\n";
        else std::cout << "X = " << flyer.getX() << "; Y = " << flyer.getY() << "; Speed = " << flyer.getSpeed() << "\n";

        outputFile << flyer.getX() << ";" << flyer.getY() << "\n";
        
        flyer.move();
        needToContinue = floor(abs(flyer.getX())) != 0 || floor(abs(flyer.getY())) != 0;
    }
    while (needToContinue);
    
    if (findArgumentInList(argc, argv, "-p") != -1) std::cout << flyer.getX() << ", " << flyer.getY() << "\n";
    else std::cout << "X = " << flyer.getX() << "; Y = " << flyer.getY() << "; Speed = " << flyer.getSpeed() << "\n";

    outputFile << flyer.getX() << ";" << flyer.getY() << "\n\n\n";

    outputFile.close();

    return 0;
}
