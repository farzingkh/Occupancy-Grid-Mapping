#include "../include/robot.h"
#include <iostream>

double Robot::forward_noise__;
double Robot::turn_noise__;

Robot::Robot(double width, double height, Sensor *sensor, Map *map, FILE *pose) : robot_width__(width), robot_height__(height), sensor__(sensor), map__(map), pose__(pose)
{
    // Randomly and uniformly position the robot inside the world
    states__.x = utility::get_random_number() * map__->get_map_width();
    states__.y = utility::get_random_number() * map__->get_map_height();
    states__.orientation = utility::get_random_number() * 2 * M_PI;
    
    std::cout << "Initialized a robot at: x=" << states__.x << " y=" << states__.y << std::endl;
}

void Robot::set_states(double new_x, double new_y, double new_orient)
{
    // replace robot states with new values
    states__.x = new_x;
    states__.y = new_y;
    states__.orientation = new_orient;
}

void Robot::set_noise(double new_f_noise, double new_t_noise, double new_s_noise)
{
    // set noises
    forward_noise__ = new_f_noise;
    turn_noise__ = new_t_noise;
    sensor__->set_noise(new_s_noise);
}

std::vector<double> Robot::sense(bool noise)
{
    //simulate the sensing using landmarks from map
    std::vector<double> lms = map__->get_landmarks();
    std::vector<double> measurements;
    // iterate through landmarks
    for (int i = 1; i < lms.size(); i += 2)
    {
        // get Euclidean distance to each landmark and add noise to simulate range finder data
        double m = sqrt(pow((lms[i - 1] - states__.x), 2) + pow((lms[i] - states__.y), 2));
        noise ? m += utility::get_gaussian_random_number(0.0, sensor__->get_sensor_noise()) : m += 0.0;
        measurements.push_back(m);
    }

    return measurements;
}

void Robot::move(double turn, double forward)
{
    // set rotation, add gaussian noise with mean of rotation bias and turn_noise as variance
    // here we assume trn bias is zero
    states__.orientation = fmod((states__.orientation + turn + utility::get_gaussian_random_number(0.0, turn_noise__)), (2 * M_PI));

    double dist = forward + utility::get_gaussian_random_number(0, forward_noise__);
    states__.x = fmod((states__.x + (dist * cos(states__.orientation))), map__->get_map_width());
    states__.y = fmod((states__.y + (dist * sin(states__.orientation))), map__->get_map_height());
}

State Robot::get_pose()
{
    // check if sensor recording file is set
    if (pose__ != nullptr)
    {
        if (std::fscanf(pose__, "%lf %lf %lf %lf", &states__.time_stamp, &states__.x, &states__.y, &states__.orientation) == EOF)
        {
            states__.time_stamp = -1;
        };

        states__.orientation = (states__.orientation /10) * (M_PI /180);
    }

    std::string pose = std::string("[") + std::string("Timestamp=") + std::to_string(states__.time_stamp) + std::string("X=") + std::to_string(states__.x) + std::string(", Y=") + std::to_string(states__.y) + std::string(" Theta=") + std::to_string(states__.orientation) + std::string("]");
    std::cout << pose << std::endl;
    return states__;
}

std::string Robot::get_sensor_readings()
{
    std::vector<double> m = sense();
    std::string readings = std::string("[");
    std::for_each(m.begin(), m.end(), [&readings](double mm) { readings += std::string(" ") + std::to_string(mm); });
    readings += std::string(" ]");

    return readings;
}

Map *Robot::get_map() { return map__; }

Sensor *Robot::get_sensor() { return sensor__; }

double Robot::get_width() { return robot_width__; }

double Robot::get_height() { return robot_height__; }