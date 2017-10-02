#ifndef ed_kinect_navigation_plugin_h_
#define ed_kinect_navigation_plugin_h_

#include <ed/plugin.h>
#include <ed/world_model.h>

#include "../../src/depth_sensor_integrator.h"

#include <ros/callback_queue.h>

class KinectNavigationPlugin : public ed::Plugin
{

public:

    KinectNavigationPlugin() {}

    virtual ~KinectNavigationPlugin() {}

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

    // --------------------

private:

    // Depth sensor integration
    DepthSensorIntegrator depth_sensor_integrator_;

};

#endif
