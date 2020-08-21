#include "../include/robot.h"
#include "../include/utility.h"

double Robot::forward_noise__;
double Robot::turn_noise__;
double Robot::sense_noise__;

Robot::Robot(double width, double height, Sensor *sensor, Map *map = nullptr, FILE *pose = nullptr) : robot_width__(width), robot_height__(height), sensor__(sensor), map__(map), pose__(pose)
{
    // Randomly and uniformly position the robot inside the world
    states__.x = utility::get_random_number() * world__.get_x();
    states__.y = utility::get_random_number() * world__.get_y();
    states__.orientation = utility::get_random_number() * 2 * M_PI;
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
        noise ? m += utility::get_gaussian_random_number(0.0, sense_noise__) : m += 0.0;
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
    states__.x = fmod((states__.x + (dist * cos(states__.orientation))), world__.get_x());
    states__.y = fmod((states__.y + (dist * sin(states__.orientation))), world__.get_y());
}

State Robot::get_pose()
{
    std::string pose = std::string("[") + std::string("X=") + std::to_string(states__.x) + std::string(", Y=") + std::to_string(states__.y) + std::string(" Theta=") + std::to_string(states__.orientation) + std::string("]");
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