#include "../include/sensor.h"
#include "../include/utility.h"
#include <iostream>

Sensor::Sensor(double min_z, double max_z, double noise, FILE *file) : min_z__(min_z), max_z__(max_z), sense_noise__(noise), file__(file)
{
    std::cout << "Initializing a range sensor ..." << std::endl;
}

void Sensor::set_noise(double noise)
{
    sense_noise__ = noise;
}

std::vector<double> Sensor::get_readings(double robot_x, double robot_y, Map *map, bool noise)
{
    std::vector<double> data(9);

    if (file__ != nullptr)
    {
        // if sensor recording exist read one line with each function call
        for (int i = 0; i < 9; ++i)
        {
            if (std::fscanf(file__, "%lf", &data[i]) == EOF)
            {
                // set data to -1 to indicate end of file
                data[i] = -1;
            }
        }
    }
    else if (map != nullptr)
    {
        // return measurements to landmarks
        std::vector<double> lms = map->get_landmarks();
        std::vector<double> measurements;
        // iterate through landmarks
        for (int i = 1; i < lms.size(); i += 2)
        {
            // get Euclidean distance to each landmark and add noise to simulate range finder data
            double m = sqrt(pow((lms[i - 1] - robot_x), 2) + pow((lms[i] - robot_y), 2));
            // add gaussian noise if noise flag is true
            noise ? m += utility::get_gaussian_random_number(0.0, sense_noise__) : m += 0.0;
            measurements.push_back(m);
        }

        return measurements;
    }
    
    // print sensor reading
    /*
    for (int i = 0; i<9 ; ++i)
    {
        std::cout << data[i];
    }
    std::cout << "\n";
    */

    return data;
}

double Sensor::forward_model(std::vector<double> readings, Map *map)
{
    // likelihood field model
    // phit
    // pmax
    // pfail
    return 0.4;
}

double Sensor::inverse_model(double robot_x, double robot_y, double robot_theta, double grid_x, double grid_y, double l_o, double l_free, double l_occ, std::vector<double>& sensor_data)
{
    // simplistic iverse model of a range sensor using beam cone model
    // measurement
    double z_k;
    // angle to measurement
    double theta_k;
    // sensor theta
    double sensor_theta;
    // minimum difference between angles
    double min_delta = -1;
    // obstacle width
    double a = 200;
    // sensor beam opening angle
    double b = 20;
    // compute distance from cell to sensor
    double r = sqrt(pow(robot_x - grid_x, 2) + pow(robot_y - grid_y, 2));
    // calculate angle between sensor reading and robot theta
    double phi = atan2(grid_y - robot_y, grid_x - robot_x) - robot_theta;
    // Sensor angles [-90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90]
    for (int i = 1; i < 9; i++)
    {
        if (i == 1)
        {
            sensor_theta = -90 * (M_PI / 180);
        }
        else if (i == 2)
        {
            sensor_theta = -37.5 * (M_PI / 180);
        }
        else if (i == 7)
        {
            sensor_theta = 37.5 * (M_PI / 180);
        }
        else if (i == 8)
        {
            sensor_theta = 90 * (M_PI / 180);
        }
        else
        {
            sensor_theta = (-37.5 + (i - 2) * 15) * (M_PI / 180);
        }

        if (fabs(phi - sensor_theta) < min_delta || min_delta == -1)
        {
            z_k = sensor_data[i];
            theta_k = sensor_theta;
            min_delta = fabs(phi - sensor_theta);
        }
    }

    // consider the three occupancy cases
    if ((r > std::min(max_z__, z_k + a / 2)) or (fabs(phi - theta_k) > b / 2) or (z_k > max_z__ ) or (z_k < min_z__))
    {
        return l_o;
    }
    else if ((z_k < max_z__) and (fabs(r - z_k) < a / 2))
    {
        return l_occ;
    }
    else if (r <= z_k)
    {
        return l_free;
    }
    else
    {
        return l_o;
    }
}

double Sensor::get_min_range() { return min_z__; }

double Sensor::get_max_range() { return max_z__; }

double Sensor::get_sensor_noise() { return sense_noise__; }
