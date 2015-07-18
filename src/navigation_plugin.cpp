#include "ed_navigation/navigation_plugin.h"

#include <ed/entity.h>
#include <ed/error_context.h>

#include <tue/config/reader.h>

// ----------------------------------------------------------------------------------------------------

void transformChull(const geo::Pose3D& pose, std::vector<geo::Vector3>& chull)
{
    for (unsigned int i = 0; i < chull.size(); ++i)
        chull[i] = pose * chull[i];
}

// --

void constructConstraint(std::vector<geo::Vector3>& chull, std::stringstream& constraint, const double& offset)
{
    if (chull.size() < 3)
    {
        std::cout << "Error: Convex hull has to consist of at least three points !!" << std::endl;
        return;
    }

    chull.push_back(chull[0]);

    double dx,dy,xi,yi,xs,ys,length;
    for (unsigned int i = 0; i < chull.size()-1; ++i)
    {
        if (i > 0)
            constraint << " and ";

        xi = chull[i].x;
        yi = chull[i].y;

        dx = chull[i+1].x - xi;
        dy = chull[i+1].y - yi;

        length = sqrt(dx * dx + dy * dy);

        xs = xi + (dy/length)*offset;
        ys = yi - (dx/length)*offset;

        constraint << "-(x-" << xs << ")*" << dy << "+(y-" << ys << ")*" << dx << " > 0";
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
        double res, min_z, max_z, unknown_obstacle_inflation;
        std::string frame_id;
        config.value("frame_id", frame_id);
        config.value("resolution", res);

        config.value("min_z", min_z);
        config.value("max_z", max_z);
        
        config.value("default_offset", default_offset_);

        if (!config.value("unknown_obstacle_inflation", unknown_obstacle_inflation, tue::OPTIONAL))
            unknown_obstacle_inflation = 0;
        
        std::cout << "Using min max " << min_z << ", " << max_z << std::endl;
        occupancy_grid_publisher_.configure(nh, res, min_z, max_z, frame_id, unknown_obstacle_inflation);

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
    constraint.precision(3); // To make sure we don't get e powers in the string
    constraint << std::fixed;
    
    bool first = true;

    for (unsigned int i = 0; i < req.entity_ids.size(); ++i)
    {
        ed::EntityConstPtr e = world_->getEntity(req.entity_ids[i]);

        std::vector<geo::Vector3> chull; // In MAP frame
        double offset = 0.0;

        if (!e)
        {
            res.error_msg = "No such entity: '" + req.entity_ids[i] + "'.";
            continue;
        }

        if (!e->has_pose())
        {
            res.error_msg = "Entity '" + req.entity_ids[i] + "' has no pose.";
            continue;
        }

        const tue::config::DataConstPointer& data = e->data();
        tue::config::Reader r(data);
        if (!data.empty() && r.readArray("areas"))
        {
            bool found_area = false;
            while(r.nextArrayItem())
            {
                std::string name;
                if (r.value("name", name) && name == req.area_names[i])
                {
                    found_area = true;

                    if (!r.readArray("shape", tue::config::OPTIONAL))
                    {
                        // If no shape specified, get the convex hull of the object
                        for (std::vector<geo::Vec2f>::const_iterator it = e->convexHull().points.begin(); it != e->convexHull().points.end(); ++it)
                            chull.push_back(geo::Vector3(it->x + e->pose().t.x, it->y + e->pose().t.y, 0.0));

                        // Default if not specified
                        if (!r.value("offset", offset, tue::config::OPTIONAL))
                            offset = default_offset_;
                    }
                    else
                    {
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
                    }
                }
            }

            if (!found_area)
                res.error_msg = "Entity '" + e->id().str() + "': area '" + req.area_names[i] + "' does not exist";

            r.endArray();
        }
        else
        {
            // Check whether the request is 'near' --> do the convex hull with default offset
            if (req.area_names[i] == "near")
            {
                for (std::vector<geo::Vec2f>::const_iterator it = e->convexHull().points.begin(); it != e->convexHull().points.end(); ++it)
                    chull.push_back(geo::Vector3(it->x + e->pose().t.x, it->y + e->pose().t.y, 0.0));
                offset = default_offset_;
            }
            else
            {
                res.error_msg = "Entity '" + e->id().str() + "': does not have an 'areas' property and the area name is not 'near'; it it '" + req.area_names[i] + "'.";
                continue;
            }
        }

        // add to constraint here
        if (!first)
            constraint << " and ";

        constructConstraint(chull, constraint, offset);

        first = false;
    }

    res.position_constraint_map_frame = constraint.str();

    return true;
}

ED_REGISTER_PLUGIN(NavigationPlugin)



