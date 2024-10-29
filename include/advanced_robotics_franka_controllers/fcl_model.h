#ifndef FCL_MODEL_H
#define FCL_MODEL_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <fcl/fcl.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <string>
#include <unordered_map>
#include <set>
#include <vector>

class FCLModel {
public:
    FCLModel(ros::NodeHandle& nh);
    bool init();
    float getDistance(const std::string& link1, const std::string& link2,
                      const geometry_msgs::Transform& relative_tf);

    void getMinDistance(std::pair<std::string, std::string> &min_dist_pair, float &min_dist);
    void getMinDistance(float &min_dist);

private:
    std::unordered_map<std::string, std::vector<std::shared_ptr<fcl::CollisionObjectf>>> coll_objects_;
    std::set<std::pair<std::string, std::string>> exclude_pairs_;
    urdf::Model model_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void loadSRDF(const std::string& srdf_content);
    std::vector<std::shared_ptr<fcl::CollisionObjectf>> createCollisionObjects(const urdf::LinkConstSharedPtr& link);
};

#endif // FCL_MODEL_H
