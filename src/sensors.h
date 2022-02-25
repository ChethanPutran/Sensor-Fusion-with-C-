#ifndef INCLUDE_NL_KALMAN_SENSORS_H
#define INCLUDE_NL_KALMAN_SENSORS_H

#include <random>
#include <vector>

// Structure for GPS measurements
struct GPS_Measurement
{
    double x, y;
};

// Structure for pose measurements
struct Gyro_Measurement
{
    double psi_dot;
};

// Structure for  Lidar measurements
struct Lidar_Measurement
{
    double range, theta;
    int id;
};

class BeaconMap;

// GPS sensor class for gps data handling
class GPS_Sensor
{
public:
    GPS_Sensor();
    void set_gps_noise_std(double std);
    void set_gps_error_prob(double prob);
    void set_gps_denied_zone(double x, double y, double r);
    GPS_Measurement generarate_gps_measurement(double sensor_x, double sensor_y);
    void reset();

private:
    std::mt19937 m_rand_gen;
    double m_noise_std;
    double m_error_prob;
    double m_denied_x;
    double m_denied_y;
    double m_denied_range;
    void reset();
};

// Gyroscopic sensor class for pose data handling
class Gyro_Sensor
{
public:
    Gyro_Sensor();
    void set_gyro_noise_std(double std);
    void set_gyro_bias(double bias);
    Gyro_Measurement generarate_gyro_measurement(double sensor_yaw_rate);
    void reset();

private:
    std::mt19937 m_rand_gen;
    double m_noise_std;
    double m_bias;
};

// Lidar sensor class for environment map handling
class Lidar_Sensor
{
public:
    Lidar_Sensor();
    void set_lidar_noise_std(double range_std, double theta_std);
    void set_lidar_max_range(double range);
    void set_lidar_da_enabled(bool id_enabled);
    std::vector<Lidar_Measurement> generarate_lidar_measurement(double sensor_x, double sensor_y, double sensor_yaw, const BeaconMap &map);
    void reset();

private:
    std::mt19937 m_rand_gen;
    double m_range_noise_std;
    double m_theta_noise_std;
    double m_max_range;
    bool m_id_enabled;
};

#endif