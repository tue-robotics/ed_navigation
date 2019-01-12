#include "ed_navigation/navigation_plugin.h"

#include <ed/entity.h>
#include <ed/error_context.h>
#include <ed/convex_hull.h>
#include <ed/convex_hull_calc.h>

#include <tue/config/reader.h>

#include <geolib/datatypes.h>
#include <geolib/Shape.h>
#include <geolib/CompositeShape.h>

#include <iomanip>

// ----------------------------------------------------------------------------------------------------

/**
 * @brief constructConstraint
 * @param ConvexHull with the points of the volume. These points should be in the correct order. This means that the
 * points should represent the border of the volume. Becuase consecutive point pairs are used to create the contstraint.
 * First point is added to the end, so also the pair 'first-last' is used.
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
 * @param shape shape of the volume in entity frame
 * @param entity_pose pose of the entity
 * @return constraint string
 */
std::string constructShapeConstraint(geo::ShapeConstPtr& shape, const geo::Pose3D& entity_pose)
{
    std::stringstream shape_constraint;

    geo::Shape shape_tr;
    shape_tr.setMesh(shape->getMesh().getTransformed(entity_pose)); // get shape in map frame.
    std::vector<geo::Vector3> points = shape_tr.getMesh().getPoints();

    // Calculate cluster convex hull
    float z_min = 1e9;
    float z_max = -1e9;

    // Calculate z_min and z_max of cluster
    std::vector<geo::Vec2f> points_2d;
    for(std::vector<geo::Vector3>::const_iterator it = points.begin(); it != points.end(); ++it)
    {
        points_2d.push_back(geo::Vec2f(it->x, it->y));

        z_min = std::min<float>(z_min, it->z);
        z_max = std::max<float>(z_max, it->z);
    }

    ed::ConvexHull chull;
    ed::convex_hull::createAbsolute(points_2d, z_min, z_max, chull);

    constructConstraint(chull, shape_constraint);
    return shape_constraint.str();
}

// ----------------------------------------------------------------------------------------------------

//
/**
 * @brief constructCompositeShapeConstraint
 * @param composite CompositeShape of the volume in entity frame
 * @param entity_pose pose of the entity
 * @return constraint string
 */
std::string constructCompositeShapeConstraint(geo::CompositeShapeConstPtr& composite, const geo::Pose3D& entity_pose)
{
    std::stringstream shape_constraint;

    bool first_sub_shape = true;

    std::vector<std::pair<geo::ShapePtr, geo::Transform> > sub_shapes = composite->getShapes();

    for (std::vector<std::pair<geo::ShapePtr, geo::Transform> >::const_iterator it = sub_shapes.begin();
         it != sub_shapes.end(); ++it)
    {
        geo::ShapeConstPtr ShapeC = boost::const_pointer_cast<geo::Shape>(it->first);
        std::string sub_shape_constraint = constructShapeConstraint(ShapeC, entity_pose * it->second.inverse());
        if (!sub_shape_constraint.empty())
        {
            if (first_sub_shape)
                first_sub_shape = false;
            else
                shape_constraint << " or ";

           shape_constraint << "(" << sub_shape_constraint << ")";
        }
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
 * @brief NavigationPlugin::srvGetGoalConstraint Create a constraint to navigate to a specific volume.
 * @param req service request
 * @param res service result
 * @return bool
 */
bool NavigationPlugin::srvGetGoalConstraint(const ed_navigation::GetGoalConstraint::Request& req, ed_navigation::GetGoalConstraint::Response& res)
{
    if (req.entity_ids.size() != req.area_names.size())
    {
        res.error_msg = "Entity ids and volume names are not from equal length";
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
        bool volume_found = false;

        if(req.area_names[i] == "near")
        {
            if (!e->shape())
            {
                res.error_msg = "Navigating to area 'near' of entity '" + req.entity_ids[i] + "' isn't possible, because it doesn't have a shape.";
                continue;
            }
            volume_found = true;
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
            std::map<std::string, geo::ShapeConstPtr> volumes = e->volumes();

            std::map<std::string, geo::ShapeConstPtr>::iterator it = volumes.find(req.area_names[i]);
            if (it != volumes.end())
            {
                volume_found = true;
                std::string shape_constraint;
                geo::CompositeShapeConstPtr composite = boost::dynamic_pointer_cast<const geo::CompositeShape>(it->second);
                if (composite)
                    shape_constraint = constructCompositeShapeConstraint(composite, e->pose());
                else
                    shape_constraint = constructShapeConstraint(it->second, e->pose());

                entity_constraint << "(" << shape_constraint << ")";
            }
        }

        if (!volume_found)
            res.error_msg = "Entity '" + e->id().str() + "': volume '" + req.area_names[i] + "' does not exist";

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



