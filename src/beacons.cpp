#include "beacons.h"
#include <cmath>
#include <random>

// Initializing the beacon vector
Beacon_Map::Beacon_Map()
{
    std::mt19937 rand_gen;
    std::uniform_real_distribution<double> pos_dis(-500.0, 500.0);
    for (int i = 0; i < 200; i++)
    {
        this->add_beacon(pos_dis(rand_gen), pos_dis(rand_gen));
    }
}

// Method to add beacon data to the vector of beacons
void Beacon_Map::add_beacon(double x, double y)
{
    this->m_beacon_map.push_back(Beacon_Data(x, y, this->m_beacon_map.size()));
}

// Method to retrieve beacon data from the vector of beacons based on beacon id
const Beacon_Data Beacon_Map::get_beacon_with_id(int id)
{
    for (const Beacon_Data &beacon_data : this->m_beacon_map)
    {
        if (beacon_data.id == id)
        {
            return beacon_data;
        }
    }
}

// Function to retrieve beacons based on  range

const std::vector<Beacon_Data> Beacon_Map::get_beacons_with_range(double x, double y, double range)
{
    std::vector<Beacon_Data> beacons;
    for (const Beacon_Data &beacon : this->m_beacon_map)
    {
        // Calculating range for each beacon
        double delta_x = beacon.x - x;
        double delta_y = beacon.y - y;
        double beacon_range = std::sqrt(delta_x * delta_x + delta_y * delta_y);

        if (beacon_range < range)
        {
            beacons.push_back(beacon);
        }
    }
    return beacons;
}
const std::vector<Beacon_Data> Beacon_Map::get_beacons()
{
    return this->m_beacon_map;
}

// Method to draw lines between vehicle and beacon
void const Beacon_Map::render(Display &disp)
{
    const std::vector<Vector2> beacon_lines = {{1, 0}, {0, 1}, {0, -1}, {1, 0}};
    // Setting yellow color
    disp.set_draw_color(255, 255, 0);
    for (const auto &beacon : this->m_beacon_map)
    {
        disp.draw_lines(offset_points(beacon_lines, Vector2(beacon.x, beacon.y)));
    }
}