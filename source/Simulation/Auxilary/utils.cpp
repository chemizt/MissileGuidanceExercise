#include "Simulation/Auxilary/utils.hpp"

double degToRad(double degrees)
{
	return degrees * M_PI / 180.0;
}

double radToDeg(double radians)
{
	return radians * 180.0 / M_PI;
}

double getAngleBetweenVectorsRad(const QVector2D& firstVector, const QVector2D& secondVector)
{
	double angle = atan2(secondVector.y(), secondVector.x()) - atan2(firstVector.y(), firstVector.x());

	// нормализуем угол в диапазон -pi..pi
	if (angle > M_PI) angle -= 2 * M_PI;
	else if (angle <= -M_PI) angle += 2 * M_PI;

	return angle;
}

void rotateVec(double angle, QVector2D& vector, const bool isRad)
{
	QTransform transform = isRad ? QTransform().rotateRadians(angle) : QTransform().rotate(angle);
	QPointF rotatedPoint = transform.map(vector.toPointF());

	vector.setX(rotatedPoint.x());
	vector.setY(rotatedPoint.y());
}

double lerp(double currX, double prevX, double prevY, double nextX, double nextY)
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
