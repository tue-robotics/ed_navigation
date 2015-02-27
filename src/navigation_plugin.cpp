#include "ed_navigation/navigation_plugin.h"

#include <ed/entity.h>
#include <tue/config/reader.h>

// ----------------------------------------------------------------------------------------------------

void transformChull(const geo::Pose3D& pose, std::vector<geo::Vector3>& chull)
{
    for (unsigned int i = 0; i < chull.size(); ++i)
        chull[i] = pose * chull[i];
}

// --

void constructConstraint(std::vector<geo::Vector3>& chull, std::stringstream& constraint)
{
    if (chull.size() < 3)
    {
        std::cout << "Error: Convex hull has to consist of at least three points !!" << std::endl;
        return;
    }

    chull.push_back(chull[0]);

    double dx,dy,xi,yi;
    for (unsigned int i = 0; i < chull.size()-1; ++i)
    {
        if (i > 0)
            constraint << " and ";

        xi = chull[i].x;
        yi = chull[i].y;
        dx = chull[i+1].x - xi;
        dy = chull[i+1].y - yi;

        constraint << "-(x-" << xi << ")*" << dy << "+(y-" << yi << ")*" << dx << "> 0";
    }
}

// ----------------------------------------------------------------------------------------------------

void NavigationPlugin::configure(tue::Configuration config)
{
    ros::NodeHandle nh("~/navigation");

    ros::AdvertiseServiceOptions opt_srv_get_goal_constraint =
            ros::AdvertiseServiceOptions::create<ed_navigation::GetGoalConstraint>(
                "get_constraint", boost::bind(&NavigationPlugin::srvGetGoalConstraint, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_get_goal_constraint_ = nh.advertiseService(opt_srv_get_goal_constraint);

    // Configure the occupancy grid publisher
    if (config.readGroup("occupancy_grid_publisher"))
    {
        double res;
        std::string frame_id;
        config.value("frame_id", frame_id);
        config.value("resolution", res);
        occupancy_grid_publisher_.configure(nh, res, frame_id);

        config.endGroup();
    }
}

// ----------------------------------------------------------------------------------------------------

void NavigationPlugin::initialize()
{

}

// ----------------------------------------------------------------------------------------------------

void NavigationPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    // Check for services
    world_ = &world;
    cb_queue_.callAvailable();

    // Publish the occupancy grid
    if (occupancy_grid_publisher_.configured())
        occupancy_grid_publisher_.publish(world);
}

// ----------------------------------------------------------------------------------------------------

bool NavigationPlugin::srvGetGoalConstraint(const ed_navigation::GetGoalConstraint::Request& req, ed_navigation::GetGoalConstraint::Response& res)
{
    if (req.entity_ids.size() != req.area_names.size())
    {
        res.error_msg = "Entity ids and area names are not from equal length";
        return true;
    }

    std::stringstream constraint;

    for (unsigned int i = 0; i < req.entity_ids.size(); ++i)
    {
        ed::EntityConstPtr e = world_->getEntity(req.entity_ids[i]);

        if (!e)
        {
            res.error_msg = "No such entity: '" + req.entity_ids[i] + "'.";
            continue;
        }

        const tue::config::DataConstPointer& data = e->data();
        if (data.empty())
        {
            res.error_msg = "Entity '" + e->id().str() + "': does not have an 'areas' property.";
            continue;
        }

        tue::config::Reader r(data);
        if (!r.readArray("areas"))
        {
            res.error_msg = "Entity '" + e->id().str() + "': does not have an 'areas' property.";
            continue;
        }

        bool found_area = false;
        while(r.nextArrayItem())
        {
            std::string name;
            if (r.value("name", name) && name == req.area_names[i])
            {
                found_area = true;
                if (!r.readArray("shape", tue::config::REQUIRED))
                {
                    res.error_msg = "Entity '" + e->id().str() + "': area '" + req.area_names[i] + "' does not have 'shape' property.";
                }
                else
                {
                    std::vector<geo::Vector3> chull;
                    while(r.nextArrayItem())
                    {
                        if (r.readGroup("box"))
                        {
                            geo::Vector3 min, max;

                            if (r.readGroup("min"))
                            {
                                r.value("x", min.x);
                                r.value("y", min.y);
                                r.value("z", min.z);
                                r.endGroup();
                            }

                            if (r.readGroup("max"))
                            {
                                r.value("x", max.x);
                                r.value("y", max.y);
                                r.value("z", max.z);
                                r.endGroup();
                            }

                            chull.push_back(min);
                            chull.push_back(geo::Vector3(max.x, min.y, 0));
                            chull.push_back(max);
                            chull.push_back(geo::Vector3(min.x, max.y, 0));

                            r.endGroup();
                        }
                    }

                    r.endArray();

                    // Transform to map frame
                    transformChull(e->pose(), chull);

                    // add to constraint here
                    if (i > 0)
                        constraint << " and ";

                    constructConstraint(chull, constraint);
                }
            }
        }

        if (!found_area)
            res.error_msg = "Entity '" + e->id().str() + "': area '" + req.area_names[i] + "' does not exist";

        r.endArray();
    }

    res.position_constraint_map_frame = constraint.str();

    return true;
}

ED_REGISTER_PLUGIN(NavigationPlugin)



