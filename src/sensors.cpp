#include "sensors.h"

// Constructor for GPS_Sensor
// Implimenting GPS Sensor methods
GPS_Sensor::GPS_Sensor()
{
    m_rand_gen = std::mt19937();
    m_noise_std = 0.0;
    m_error_prob = 0.0;
    m_denied_x = 0.0;
    m_denied_y = 0.0;
    m_denied_range = -1.0;
}

void GPS_Sensor::set_gps_noise_std(double std)
{
    this->m_noise_std = std;
}
void GPS_Sensor::set_gps_error_prob(double prob)
{
    this->m_error_prob = prob;
}
void GPS_Sensor::set_gps_denied_zone(double x, double y, double range)
{
    this->m_denied_x = x;
    this->m_denied_y = y;
    this->m_denied_range = range;
}
GPS_Measurement GPS_Sensor::generarate_gps_measurement(double sensor_x, double sensor_y)
{
    GPS_Measurement gps_measurement;
    double mean = 0.0;
    std::normal_distribution<double> gps_pos_dis = std::normal_distribution(mean, m_noise_std);
    std::uniform_real_distribution<double> gps_error_dis(0.0, 1.0);
    gps_measurement.x = sensor_x + gps_pos_dis(this->m_rand_gen);
    gps_measurement.y = sensor_y + gps_pos_dis(this->m_rand_gen);

    if (gps_error_dis(this->m_rand_gen) < this->m_error_prob)
    {
        gps_measurement.x = 0;
        gps_measurement.y = 0;
    }

    double delta_x = sensor_x - this->m_denied_x;
    double delta_y = sensor_y - this->m_denied_y;

    double range = sqrt(delta_x * delta_x + delta_y * delta_y);
    if (range < this->m_denied_range)
    {
        gps_measurement.x = 0;
        gps_measurement.y = 0;
    }
    return gps_measurement;
}

// Gyro Sensor
Gyro_Sensor::Gyro_Sensor() : m_rand_gen(std::mt19937()), m_noise_std(0.0), m_bias(0.0) {}

void Gyro_Sensor::reset()
{
    this->m_rand_gen = std::mt19937();
}
void Gyro_Sensor::set_gyro_noise_std(double std)
{
    this->m_noise_std = std;
}
void Gyro_Sensor::set_gyro_bias(double bias)
{
    this->m_bias = bias;
}
Gyro_Measurement Gyro_Sensor::generarate_gyro_measurement(double sensor_yaw_rate)
{
    Gyro_Measurement gyro_measurement;
    std::normal_distribution<double> gyro_dis(0.0, this->m_noise_std);
    gyro_measurement.psi_dot = sensor_yaw_rate + this->m_bias + gyro_dis(this->m_rand_gen);
    return gyro_measurement;
}

// Lidar Sensor
Lidar_Sensor::Lidar_Sensor() : m_rand_gen(std::mt19937()), m_range_noise_std(0.0), m_theta_noise_std(0.0), m_max_range(90.0), m_id_enabled(true) {}

void Lidar_Sensor::reset()
{
    this->m_rand_gen = std::mt19937();
}

void Lidar_Sensor::set_lidar_noise_std(double range_std, double theta_std)
{
    this->m_range_noise_std = range_std;
    this->m_theta_noise_std = theta_std;
}

void Lidar_Sensor::set_lidar_max_range(double range)
{
    this->m_max_range = range;
}

void Lidar_Sensor::set_lidar_da_enabled(bool id_enabled)
{
    this->m_id_enabled = id_enabled;
}

std::vector<Lidar_Measurement> Lidar_Sensor::generarate_lidar_measurement(double sensor_x, double sensor_y, double sensor_yaw, const BeaconMap &map)
{
    std::vector<Lidar_Measurement> lidar_measurement;
    std::normal_distribution<double> lidar_theta_dis(0.0, m_theta_noise_std);
    std::normal_distribution<double> lidar_range_dis(0.0, m_range_noise_std);
    for (const auto &beacon : map.getBeacons())
    {
        double delta_x = beacon.x - sensor_x;
        double delta_y = beacon.y - sensor_y;
        double theta = wrapAngle(atan2(delta_y, delta_x) - sensor_yaw);
        double beacon_range = std::sqrt(delta_x * delta_x + delta_y * delta_y);

        if (beacon_range < m_max_range)
        {
            Lidar_Measurement beacon_meas;
            beacon_meas.range = std::abs(beacon_range + lidar_range_dis(this->m_rand_gen));
            beacon_meas.theta = wrapAngle(theta + lidar_theta_dis(this->m_rand_gen));
            beacon_meas.id = (this->m_id_enabled ? beacon.id : -1);
            lidar_measurement.push_back(beacon_meas);
        }
    }
    return lidar_measurement;
}
