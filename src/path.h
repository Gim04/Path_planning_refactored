#ifndef path.h
#include "CDT.h"
#include <vector>

struct CustomPoint2D{

    double data[2];

};

struct CustomEdge{

    std::pair<std::size_t, std::size_t> vertices;

};
#endif