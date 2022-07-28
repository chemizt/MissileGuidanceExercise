#include <QTransform>
#include "Simulation/SimObjects/MovingObject.hpp"

using namespace std::chrono;
using std::mt19937_64;
using std::random_device;
using std::uniform_real_distribution;

MovingObject::MovingObject(double initialSpeed, double initialX, double initialY)
{
	_coordinates = QVector2D(initialX, initialY);
	_velocity = QVector2D(0, initialSpeed); // создаём вектор скорости, сонаправленный с осью Y

	_actingVectors.insert("velocity", _velocity);

	random_device rD;
	mt19937_64::result_type seed = rD() ^
		((mt19937_64::result_type) duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count() +
		(mt19937_64::result_type) duration_cast<microseconds>(high_resolution_clock::now().time_since_epoch()).count());

	_leMersenneTwister = mt19937_64(seed);
}

double MovingObject::_getRandomInRange(double minValue, double maxValue)
{
	uniform_real_distribution<double> distribution(minValue, maxValue);

	return distribution(_leMersenneTwister);
}

void MovingObject::_rotateActingVectors(double angle)
{
	foreach(QVector2D value, _actingVectors)
	{
		QTransform transform = QTransform().rotate(angle);
		QPointF rotatedPoint = transform.map(value.toPointF());
		
		value.setX(rotatedPoint.x());
		value.setY(rotatedPoint.y());
	}
}
