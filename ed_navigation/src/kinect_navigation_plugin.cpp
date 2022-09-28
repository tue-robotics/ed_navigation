#include "ed_navigation/kinect_navigation_plugin.h"

#include <ed/entity.h>
#include <ed/error_context.h>

#include <tue/config/reader.h>

#include <iomanip>

// ----------------------------------------------------------------------------------------------------

void KinectNavigationPlugin::configure(tue::Configuration config)
{
    // configure the depth_sensor integration
    if (config.readGroup("depth_sensor_integration"))
    {
        depth_sensor_integrator_.initialize(config);
        config.endGroup();
    }
}

// ----------------------------------------------------------------------------------------------------

void KinectNavigationPlugin::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void KinectNavigationPlugin::process(const ed::WorldModel& /*world*/, ed::UpdateRequest& /*req*/)
{
    // Publish the occupancy grid
    if (depth_sensor_integrator_.isInitialized())
        depth_sensor_integrator_.update();
}

// ----------------------------------------------------------------------------------------------------


ED_REGISTER_PLUGIN(KinectNavigationPlugin)
