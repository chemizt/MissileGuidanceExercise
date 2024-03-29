#include "Simulation/SimObjects/MovingObject.hpp"
#include "Simulation/Auxilary/utils.hpp"

using namespace std::chrono;
using std::mt19937_64;
using std::random_device;
using std::uniform_real_distribution;

MovingObject::MovingObject(double initialSpeed, double initialX, double initialY)
{
	_coordinates = QVector2D(initialX, initialY);

	_actingVectors.try_emplace("velocity", QVector2D(0, initialSpeed));

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

void MovingObject::_rotateActingVectorsRad(double angle)
{
	for (auto& [key, entry] : _actingVectors)
		rotateVec(angle, entry);
}
