
#include "CDT.h"
#include <vector>

struct CustomPoint2D{

    double data[2];

};

struct CustomEdge{

    std::pair<std::size_t, std::size_t> vertices;

};

void spapi(int grado, const std::vector<double>& valori, std::vector<double>& spline);

double fnval(const std::vector<double>& spline, double t);
