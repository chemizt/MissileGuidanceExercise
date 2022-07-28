#ifndef MOVOBJ_HDR_IG
#define MOVOBJ_HDR_IG

#include <QMap>
#include <QString>
#include <QVector2D>
#include <QPointF>
#include <random>

class MovingObject // базовый класс движущихся объектов - имеет только скорость и направление движения
{
	public:
		MovingObject() = delete;
		MovingObject(double initialSpeed, double initialX, double initialY);
		double getSpeed() { return _velocity.length(); }
		double getX() { return _coordinates.x(); }
		double getY() { return _coordinates.y(); }
		QVector2D getCoordinates() { return _coordinates; }
		QVector2D getVelocity() { return _velocity; }

	protected:
		QMap<QString, QVector2D> _actingVectors;
		QVector2D _coordinates;
		QVector2D _velocity;
		std::mt19937_64 _leMersenneTwister;
		double _getRandomInRange(double minValue, double maxValue);
		void _rotateActingVectors(double angle); // поворачивает действующие на объект векторы в соответствии с углом, на который поворачивает объект
};

#endif // MOVOBJ_HDR_IG
