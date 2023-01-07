#ifndef ED_NAVIGATION_OCCUPANCY_GRID_PUBLISHER_H_
#define ED_NAVIGATION_OCCUPANCY_GRID_PUBLISHER_H_

#include <ed/plugin.h>
#include <ed/types.h>

#include <geolib/datatypes.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <tue/config/configuration.h>

#include "map.h"

namespace ed
{

namespace navigation
{

class OccupancyGridPublisher
{

public:

    OccupancyGridPublisher() : configured_(false) {}

    ~OccupancyGridPublisher() = default;

    /**
     * @brief configure configure hook
     * @param nh NodeHandle to use
     * @param config Configuration object with the following parameters:
     * parametergroup: occupancy_grid_publisher
     * parameters:
     *      frame_id (recommended: map): Frame id of the published costmap
     *      resolution (recommended: 0.05): Resolution of the published costmap
     *      min_z (recommended: 0.025): Only shaped above this height are taken into account
     *      max_z (recommended: 1.8): Only shaped above this height are taken into account
     *      min_map_size_x (optional, default 20): Minimal map size in x-direction, any entities with a shape outside this size will enlarge the map.
     *      min_map_size_y (optional, default 20): Minimal map size in y-direction, any entities with a shape outside this size will enlarge the map.
     */
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

} // namespace navigation

} // namespace ed

#endif // ED_NAVIGATION_OCCUPANCY_GRID_PUBLISHER_H_
