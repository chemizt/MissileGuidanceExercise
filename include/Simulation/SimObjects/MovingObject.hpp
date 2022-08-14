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
		double getSpeed() { return _actingVectors.at("velocity").length(); }
		double getX() { return _coordinates.x(); }
		double getY() { return _coordinates.y(); }
		QVector2D getCoordinates() { return _coordinates; }
		QVector2D getVelocity() { return _actingVectors.at("velocity"); }

	protected:
		std::unordered_map<QString, QVector2D> _actingVectors;
		QVector2D _coordinates;
		std::mt19937_64 _leMersenneTwister;
		double timeSinceBirth{0};
		double _getRandomInRange(double minValue, double maxValue);
		void _rotateActingVectorsRad(double angle); // поворачивает действующие на объект векторы в соответствии с углом, на который поворачивает объект
};

#endif // MOVOBJ_HDR_IG
