#include "../include/map.h"
#include <iostream>

Map::Map(double map_width, double map_height, double grid_width, double grid_height) : map_width__(map_width), map_height__(map_height), grid_width__(grid_width), grid_height__(grid_height)
{
    // initialize a map coonsidering map and grid cell dimensions
    std::cout << "Initializing a Map of size " << map_width << "x" << map_height << " ..." << std::endl;
    data__ = std::vector<std::vector<double>>((map_height__ / grid_height__), std::vector<double>(map_width__ / grid_width__, 0.0));
}

void Map::set_landmarks(double x, double y)
{
    landmarks__.push_back(x);
    landmarks__.push_back(y);
    // check invariant condition
    assert(landmarks__.size() % 2 == 0);
    std::cout << "[Landmark] x:" << x << " y:" << y << std::endl;
}

std::vector<double> Map::get_landmarks()
{
    return landmarks__;
}

std::vector<double> &Map::operator[](int x)
{
    return data__[x];
}

void Map::set_occupancy(double x, double y, double log_odds)
{
    if (x >= 0 && x < (map_width__ / grid_width__) && y >= 0 && y < (map_height__ / grid_height__))
    {
        data__[y][x] = log_odds;
    }
    else
    {

        std::cout << "Invalid indeces!" << std::endl;
    }
}

double Map::get_occupancy(double x, double y)
{
    if (x >= 0 && x < (map_width__ / grid_width__) && y >= 0 && y < (map_height__ / grid_height__))
    {
        return data__[y][x];
    }
    else
    {

        std::cout << "Invalid indeces!" << std::endl;
        return 0;
    }
}

double Map::get_map_width()
{
    return map_width__;
}

double Map::get_map_height()
{
    return map_height__;
}

double Map::get_grid_width()
{
    return grid_width__;
}

double Map::get_grid_height()
{
    return grid_height__;
}
