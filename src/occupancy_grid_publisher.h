#ifndef occupancy_grid_publisher_h_
#define occupancy_grid_publisher_h_

#include <ros/ros.h>
#include <geolib/datatypes.h>
#include <ed/plugin.h>
#include <ed/types.h>

#include "map.h"

class OccupancyGridPublisher
{

public:

    OccupancyGridPublisher() : configured_(false) {}

    void configure(ros::NodeHandle& nh, tue::Configuration config, const double &res, const double& min_z, const double& max_z,
                   const std::string &frame_id, double unknown_obstacle_inflation);

    void publish(const ed::WorldModel& world);

    bool configured() { return configured_; }

private:

    bool getMapData(const ed::WorldModel& world, std::vector<ed::EntityConstPtr>& entities_to_be_projected);

    void updateMap(const ed::EntityConstPtr& e, Map& map);

    void publishMapMsg (const Map& map);

    ros::Publisher map_pub_;

    //! Configurarable parameters  --------------

    bool configured_;

    std::string frame_id_;

    double min_z_, max_z_;

    // Inflation of unknown obstacles (in meters)
    double unknown_obstacle_inflation_;

    bool convex_hull_enabled_;

    // Map
    Map map_;
};

#endif
