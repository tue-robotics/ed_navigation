#ifndef ed_navigation_plugin_h_
#define ed_navigation_plugin_h_

#include <ed/plugin.h>
#include <ed/world_model.h>

#include <ed_navigation/GetGoalConstraint.h>

#include "../../src/occupancy_grid_publisher.h"

#include <ros/callback_queue.h>

/**
 * @brief The NavigationPlugin class
 * ED plugin for publishing a map for navigation
 * The objects, walls and furniture, in the current world model, which are inbetween specified heights, are down-projected. The result is published as a occupancy grid
 */
class NavigationPlugin : public ed::Plugin
{

public:

    /**
     * @brief constructor
     */
    NavigationPlugin() {}

    /**
     * @brief destructor
     */
    virtual ~NavigationPlugin() {}

    /**
     * @brief configure
     * @param config
     * parametergroup: occupancy_grid_publisher
     * parameters:
     *      resolution: double, resolution of the occupancy grid (meters)
     *      frame_id: id of the frame, probaly '/map'
     *      min_z: double, only use the volume of object higher than min_z (meters)
     *      max_z: double, only use the volume of object lower than max_z (meters)
     *      default_offset: ???
     */
    void configure(tue::Configuration config);

    /**
     * @brief initialize
     */
    void initialize();

    /**
     * @brief process
     * @param world
     * @param req
     */
    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

    // --------------------

private:

    /**
     * @brief Get goal constraint for navigation to objects in ED by a ros service call
     * @param req   ed_navigation::GetGoalConstraint::Request&
     * @param res   ed_navigation::GetGoalConstraint::Response&
     * @return bool
     */
    bool srvGetGoalConstraint(const ed_navigation::GetGoalConstraint::Request& req, ed_navigation::GetGoalConstraint::Response& res);

    // Services
    ros::ServiceServer srv_get_goal_constraint_;
    ros::CallbackQueue cb_queue_;

    const ed::WorldModel* world_;

    // Occupancy grid publisher
    OccupancyGridPublisher occupancy_grid_publisher_;

    // Parameters
    std::string goal_constraint_service_name_;
    
    // Default
    double default_offset_;
    double room_offset_; // offset to a room. Room is defined to the edges, but you want the robot to drive more into
                         // the room, instead of really standing in the door/on the room edge. So offset is the distance
                         // how much the base_link of the robot should be more into the room.

};

#endif
