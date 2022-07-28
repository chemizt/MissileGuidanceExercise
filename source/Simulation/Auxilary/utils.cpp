#include "Simulation/Auxilary/utils.hpp"

int findArgumentInList(unsigned int argc, char const* argv[], std::string arg)
{
	int result = -1;

	for (unsigned int i = 0; (i < argc) && (result == -1); i++)
	{
		if (argv[i] == arg) result = i;
	}

	return result;
}

double convertDegreesToRadians(double degrees)
{
	return degrees * M_PI / 180.0;
}

double convertRadiansToDegrees(double radians)
{
	return radians * 180.0 / M_PI;
}

double getAngleBetweenVectors(const QVector2D& firstVector, const QVector2D& secondVector)
{
	double angle = atan2(secondVector.y(), secondVector.x()) - atan2(firstVector.y(), firstVector.x());

	// нормализуем угол в диапазон -pi..pi
	if (angle > M_PI) angle -= 2 * M_PI;
	else if (angle <= -M_PI) angle += 2 * M_PI;

	return convertRadiansToDegrees(angle);
}

double interpolateWithLinearInterpolation(double currX, double prevX, double prevY, double nextX, double nextY)
{
	return prevY + (currX - prevX) * (nextY - prevY) / (nextX - prevX);
}

string convertDoubleToStringWithPrecision(double dbl, int precision, bool changeDecimal)
{
	stringstream s;

	if (changeDecimal) s.imbue(locale(locale::classic(), new Comma));
	s << fixed << setprecision(precision);
	s << dbl;

	return s.str();
}
