#include "ed_navigation/navigation_plugin.h"

#include <ed/entity.h>
#include <ed/error_context.h>
#include <ed/convex_hull.h>
#include <ed/convex_hull_calc.h>

#include <tue/config/reader.h>

#include <geolib/datatypes.h>
#include <geolib/Shape.h>

#include <iomanip>

// ----------------------------------------------------------------------------------------------------

/**
 * @brief constructConstraint
 * @param ConvexHull with the points of the area. These points should be in the correct order. This means that the points should represent the border of the area.
 * Becuase consecutive point pairs are used to create the contstraint. First point is added to the end, so also the pair 'first-last' is used.
 * @param constraint string with the contraint
 * @param offset offset to the constraint
 */
void constructConstraint(ed::ConvexHull& chull, std::stringstream& constraint, double offset = 0)
{
    if (chull.points.size() < 3)
    {
        std::cout << "Error: Convex hull has to consist of at least three points !!" << std::endl;
        return;
    }

    std::vector<geo::Vec2f> points = chull.points;
    points.push_back(points[0]);

    // To make sure we don't get e powers in the string
    constraint << std::fixed << std::setprecision(6);

    constraint << "(";

    double dx,dy,xi,yi,xs,ys,length;
    for (unsigned int i = 0; i < points.size()-1; ++i)
    {
        if (i > 0)
            constraint << " and ";

        xi = points[i].x;
        yi = points[i].y;

        dx = points[i+1].x - xi;
        dy = points[i+1].y - yi;

        length = sqrt(dx * dx + dy * dy);

        xs = xi + (dy/length)*offset;
        ys = yi - (dx/length)*offset;

        constraint << "-(x-" << xs << ")*" << dy << "+(y-" << ys << ")*" << dx << " > 0";
    }

    constraint << ")";
}

// ----------------------------------------------------------------------------------------------------

//
/**
 * @brief constructShapeConstraint
 * @param r r must be currently reading an array of shapes
 * @param entity_pose pose of the entity
 * @return constraint string
 */
std::string constructShapeConstraint(geo::ShapeConstPtr& shape, const geo::Pose3D& entity_pose)
{
    std::stringstream shape_constraint;

    bool first_sub_shape = true;
    geo::Shape shape_tr;
    shape_tr.setMesh(shape->getMesh().getTransformed(entity_pose));
    std::vector<geo::Vector3> points = shape_tr.getMesh().getPoints();

    // Calculate cluster convex hull
    float z_min = 1e9;
    float z_max = -1e9;

    // Calculate z_min and z_max of cluster
    std::vector<geo::Vec2f> points_2d(points.size());
    for(int j = 0; j < points.size(); ++j)
    {
        const geo::Vec3& p = points[j];

        // Transform sensor point to map frame
        geo::Vector3 p_map = entity_pose * p;

        points_2d[j] = geo::Vec2f(p_map.x, p_map.y);

        z_min = std::min<float>(z_min, p_map.z);
        z_max = std::max<float>(z_max, p_map.z);
    }

    ed::ConvexHull chull;
    ed::convex_hull::createAbsolute(points_2d, z_min, z_max, chull);

    std::stringstream sub_shape_constraint;
    constructConstraint(chull, sub_shape_constraint);

    if (!sub_shape_constraint.str().empty())
    {
        if (first_sub_shape)
            first_sub_shape = false;
        else
            shape_constraint << " or ";

       shape_constraint << "(" << sub_shape_constraint.str() << ")";
    }

    return shape_constraint.str();
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

        if (!config.value("unknown_obstacle_inflation", unknown_obstacle_inflation, tue::config::OPTIONAL))
            unknown_obstacle_inflation = 0;

        std::cout << "Using min max " << min_z << ", " << max_z << std::endl;
        occupancy_grid_publisher_.configure(nh, config, res, min_z, max_z, frame_id, unknown_obstacle_inflation);

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

/**
 * @brief NavigationPlugin::srvGetGoalConstraint Create a constraint to navigate to a specific area.
 * @param req service request
 * @param res service result
 * @return bool
 */
bool NavigationPlugin::srvGetGoalConstraint(const ed_navigation::GetGoalConstraint::Request& req, ed_navigation::GetGoalConstraint::Response& res)
{
    if (req.entity_ids.size() != req.area_names.size())
    {
        res.error_msg = "Entity ids and area names are not from equal length";
        return true;
    }

    std::stringstream constraint;
    constraint.precision(6); // To make sure we don't get e powers in the string
    constraint << std::fixed;

    bool first = true;

    for (unsigned int i = 0; i < req.entity_ids.size(); ++i)
    {
        ed::EntityConstPtr e = world_->getEntity(req.entity_ids[i]);

        double offset = 0.;

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

        std::stringstream entity_constraint;
        bool area_found = false;

        if(req.area_names[i] == "near")
        {
            area_found = true;
            std::vector<geo::Vec2f> points;

            for (std::vector<geo::Vec2f>::const_iterator it = e->convexHull().points.begin(); it != e->convexHull().points.end(); ++it)
                points.push_back(geo::Vec2f(it->x + e->pose().t.x, it->y + e->pose().t.y));
            ed::ConvexHull chull; // In MAP frame
            ed::convex_hull::createAbsolute(points, 0., 0.1, chull);
            offset = default_offset_;

            constructConstraint(chull, entity_constraint, offset);
        }
        else
        {

            std::map<std::string, geo::ShapeConstPtr> areas = e->areas();

            for (std::map<std::string, geo::ShapeConstPtr>::iterator it = areas.find(req.area_names[i]); it != areas.end(); ++it)
            {
                area_found = true;
                std::string shape_constraint = constructShapeConstraint(it->second, e->pose());
                entity_constraint << "(" << shape_constraint << ")";
            }
        }

        if (!area_found)
            res.error_msg = "Entity '" + e->id().str() + "': area '" + req.area_names[i] + "' does not exist";

        // add to constraint here
        if (first)
            first = false;
        else
            constraint << " and ";

        constraint << "(" << entity_constraint.str() << ")";
    }


    res.position_constraint_map_frame = constraint.str();

    return true;
}

ED_REGISTER_PLUGIN(NavigationPlugin)



