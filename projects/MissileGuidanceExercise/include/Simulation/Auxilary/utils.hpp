#ifndef UTILS_HDR_IG
#define UTILS_HDR_IG

#define _USE_MATH_DEFINES

#include <cmath>
#include <random>
#include <string>
#include <sstream>
#include <locale>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <charconv>
#include <QMap>
#include <QString>
#include <QVector2D>
#include <QTransform>

#define STANDARD_PRECISION 5

using namespace std::chrono;
using std::mt19937_64;
using std::random_device;
using std::uniform_real_distribution;
using std::string;
using std::stringstream;
using std::fixed;
using std::setprecision;
using std::locale;
using std::map;
using std::ofstream;
using std::stod;
using std::ios_base;
using std::vector;
using std::to_string;

struct Comma final : std::numpunct<char> // inspired by (copy-typed from) https://stackoverflow.com/a/42331536
{
	char do_decimal_point() const override { return ','; }
};

double degToRad(double degrees);

double radToDeg(double radians);

double getAngleBetweenVectorsRad(const QVector2D& firstVector, const QVector2D& secondVector);

void rotateVec(double angle, QVector2D& vector, const bool isRad = true);

double lerp(double currX, double prevX, double prevY, double nextX, double nextY);

string convertDoubleToStringWithPrecision(double dbl, int precision = STANDARD_PRECISION, bool changeDecimal = true);

#endif
