#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include "matplotlibcpp.h"
#include "map.h"

namespace Visualization
{
    namespace plt = matplotlibcpp;
    
    void mapShow(Map *map);
};

#endif /* VISUALIZATION_H */