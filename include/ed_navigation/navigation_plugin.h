#ifndef ed_navigation_plugin_h_
#define ed_navigation_plugin_h_

#include <ed/plugin.h>
#include <ed/world_model.h>

#include <ed_navigation/GetGoalConstraint.h>

#include "../../src/occupancy_grid_publisher.h"

#include <ros/callback_queue.h>

class NavigationPlugin : public ed::Plugin
{

public:

    NavigationPlugin() {}

    virtual ~NavigationPlugin() {}

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);    

    // --------------------

private:

    bool srvGetGoalConstraint(const ed_navigation::GetGoalConstraint::Request& req, ed_navigation::GetGoalConstraint::Response& res);

    // Services
    ros::ServiceServer srv_get_goal_constraint_;
    ros::CallbackQueue cb_queue_;

    const ed::WorldModel* world_;
\
    // Occupancy grid publisher
    OccupancyGridPublisher occupancy_grid_publisher_;

    // Parameters
    std::string goal_constraint_service_name_;

};

#endif
