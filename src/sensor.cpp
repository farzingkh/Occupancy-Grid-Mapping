#include "../include/sensor.h"
#include <iostream>

Sensor::Sensor(double min_z, double max_z, double noise, FILE *file) : min_z__(min_z), max_z__(max_z), sense_noise__(noise), file__(file) {}

void Sensor::set_noise(double noise)
{
    sense_noise__ = noise;
}

std::vector<double> get_readings(double robot_x, double robot_y, Map *map, bool noise)
{
    std::vector<double> data(8);

    if (file != nullptr)
    {
        // if sensor recording exist read one line with each function call
        for (int i = 0; i < 8; ++i)
        {
            std::fscanf(file, "%lf", &data[i]);
        }
    }
    else if (map != nullptr)
    {
        // return measurements to landmarks
        std::vector<double> lms = map__->get_landmarks();
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

double Sensor::inverse_model(double robot_x, double robot_y, double robot_theta, double grid_x, double grid_y)
{
    // simplistic iverse model of a range sensor
    return 0.4;
}

double Sensor::get_min_range() { return min_z__; }

double Sensor::get_max_range() { return max_z__; }

double Sensor::get_sensor_noise() { return sensor_noise__; }

