#ifndef ed_kinect_navigation_plugin_h_
#define ed_kinect_navigation_plugin_h_

#include <ed/plugin.h>
#include <ed/world_model.h>

#include "../../src/depth_sensor_integrator.h"

//!
//! \brief The KinectNavigationPlugin class
//! \details ED plugin of the DepthSensorIntergration, which is used to detect obstacles
//!
class KinectNavigationPlugin : public ed::Plugin
{

public:

    //!
    //! \brief constructor
    //!
    KinectNavigationPlugin() {}

    //!
    //! \brief destructor
    //!
    virtual ~KinectNavigationPlugin() {}

    //!
    //! \brief configure
    //! \param config tue::Configuration
    //!
    void configure(tue::Configuration config);

    //!
    //! \brief initialize
    //!
    void initialize();

    //!
    //! \brief process
    //! \param world ed::WorldModel&
    //! \param req ed::UpdateRequest&
    //!
    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

    // --------------------

private:

    //!
    //! \brief depth_sensor_integrator_
    //!
    DepthSensorIntegrator depth_sensor_integrator_;

};

#endif
