#ifndef UTILS_HDR_IG
#define UTILS_HDR_IG

#include <random>
#include <string>
#include <locale>
#include <chrono>

#pragma region ARGUMENT_KEYS
#define SPEED_KEY "-s"
#define ACCELERATION_KEY "-a"
#define OUTPUT_TO_FILE_KEY "-of"
#define OUTPUT_COORDS_ONLY_KEY "-co"
#pragma endregion

using namespace std::chrono;
using std::mt19937_64;
using std::random_device;
using std::uniform_real_distribution;

struct Comma final : std::numpunct<char> // inspired by (copy-typed from) https://stackoverflow.com/a/42331536
{
    char do_decimal_point() const override { return ','; }
};

double getRandomInRange(double minValue, double maxValue)
{
    random_device rD;
    mt19937_64::result_type seed = rD() ^ 
    ((mt19937_64::result_type) duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count() + 
    (mt19937_64::result_type) duration_cast<microseconds>(high_resolution_clock::now().time_since_epoch()).count());

    mt19937_64 rNG(seed);
    uniform_real_distribution<double> distribution(minValue, maxValue);

    return distribution(rNG);
}

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

#endif
