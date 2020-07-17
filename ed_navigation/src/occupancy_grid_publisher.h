#ifndef occupancy_grid_publisher_h_
#define occupancy_grid_publisher_h_

#include <ed/plugin.h>
#include <ed/types.h>

#include <geolib/datatypes.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <tue/config/configuration.h>

#include "map.h"

class OccupancyGridPublisher
{

public:

    OccupancyGridPublisher() : configured_(false) {}

    void configure(ros::NodeHandle& nh, tue::Configuration config);

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

    double min_map_size_x_, min_map_size_y_;

    // Map
    Map map_;
};

#endif
