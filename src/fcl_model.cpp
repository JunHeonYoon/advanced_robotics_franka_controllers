#include "advanced_robotics_franka_controllers/fcl_model.h"
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/geometry/shape/cylinder.h>
#include <tinyxml2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace tinyxml2;

FCLModel::FCLModel(ros::NodeHandle& nh) : tf_listener_(tf_buffer_) {}

bool FCLModel::init() {
    ros::NodeHandle nh("~");

    std::string urdf;
    nh.param<std::string>("robot_description", urdf, "");
    if (urdf.empty() || !model_.initString(urdf)) {
        ROS_ERROR("Failed to parse URDF.");
        return false;
    }

    std::string srdf;
    nh.param<std::string>("robot_description_semantic", srdf, "");
    if (srdf.empty()) {
        ROS_ERROR("Failed to get SRDF.");
        return false;
    }
    loadSRDF(srdf);

    for (auto& link : model_.links_) {
        auto objs = createCollisionObjects(link.second);
        if (!objs.empty()) {
            coll_objects_[link.first] = objs;
        }
    }

    return true;
}

void FCLModel::loadSRDF(const std::string& srdf_content) {
    XMLDocument doc;
    if (doc.Parse(srdf_content.c_str()) != XML_SUCCESS) {
        ROS_ERROR("Failed to parse SRDF.");
        return;
    }

    for (XMLElement* elem = doc.RootElement()->FirstChildElement("disable_collisions"); elem != nullptr;
         elem = elem->NextSiblingElement("disable_collisions")) {
        exclude_pairs_.emplace(elem->Attribute("link1"), elem->Attribute("link2"));
    }
}

std::vector<std::shared_ptr<fcl::CollisionObjectf>> FCLModel::createCollisionObjects(const urdf::LinkConstSharedPtr& link) {
    std::vector<std::shared_ptr<fcl::CollisionObjectf>> objs;

    for (const auto& coll : link->collision_array) {
        std::shared_ptr<fcl::CollisionGeometryf> geom;

        if (coll->geometry->type == urdf::Geometry::BOX) {
            urdf::Box* box = dynamic_cast<urdf::Box*>(coll->geometry.get());
            geom = std::make_shared<fcl::Boxf>(box->dim.x, box->dim.y, box->dim.z);
        } else if (coll->geometry->type == urdf::Geometry::SPHERE) {
            urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(coll->geometry.get());
            geom = std::make_shared<fcl::Spheref>(sphere->radius);
        } else if (coll->geometry->type == urdf::Geometry::CYLINDER) {
            urdf::Cylinder* cylinder = dynamic_cast<urdf::Cylinder*>(coll->geometry.get());
            geom = std::make_shared<fcl::Cylinderf>(cylinder->radius, cylinder->length);
        }

        if (geom) {
            objs.push_back(std::make_shared<fcl::CollisionObjectf>(geom));
        }
    }

    return objs;
}

float FCLModel::getDistance(const std::string& link1, const std::string& link2,
                            const geometry_msgs::Transform& relative_tf) {
    if (exclude_pairs_.count({link1, link2}) || exclude_pairs_.count({link2, link1})) {
        ROS_INFO("Skipping excluded pair: %s, %s", link1.c_str(), link2.c_str());
        return -1.0;
    }

    auto& objs1 = coll_objects_[link1];
    auto& objs2 = coll_objects_[link2];
    float min_dist = std::numeric_limits<float>::infinity();

    for (const auto& obj1 : objs1) {
        for (const auto& obj2 : objs2) {
            fcl::Transform3f fcl_tf;
            fcl_tf.translation() = fcl::Vector3f(relative_tf.translation.x, relative_tf.translation.y, relative_tf.translation.z);
            fcl_tf.linear() = fcl::Quaternionf(relative_tf.rotation.w, relative_tf.rotation.x, relative_tf.rotation.y, relative_tf.rotation.z).toRotationMatrix();

            obj1->setTransform(fcl::Transform3f::Identity());
            obj2->setTransform(fcl_tf);

            fcl::DistanceRequestf req;
            fcl::DistanceResultf res;
            fcl::distance(obj1.get(), obj2.get(), req, res);

            if (res.min_distance < min_dist) {
                min_dist = res.min_distance;
            }
        }
    }

    return min_dist == std::numeric_limits<float>::infinity() ? -1.0 : min_dist;
}

void FCLModel::updateDistances() {
    ros::Rate rate(1000);  // 1000 Hz
    while (ros::ok()) {
        for (auto& link1 : coll_objects_) {
            for (auto& link2 : coll_objects_) {
                if (link1.first >= link2.first) continue;
                try {
                    geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(link1.first, link2.first, ros::Time(0));
                    float distance = getDistance(link1.first, link2.first, tf.transform);
                    if (distance >= 0) {
                        ROS_INFO("Distance between %s and %s: %f", link1.first.c_str(), link2.first.c_str(), distance);
                    }
                } catch (tf2::TransformException& ex) {
                    ROS_WARN("%s", ex.what());
                    ros::Duration(0.1).sleep();
                    continue;
                }
            }
        }
        rate.sleep();
    }
}
