#include "archlib/ROSComponent.hpp"

namespace arch {
	ROSComponent::ROSComponent(int &argc, char **argv, const std::string &name) : rosComponentDescriptor() {
        ros::init(argc, argv, name, ros::init_options::NoSigintHandler); //Configure node name and sets commnd line arguments
        std::string node_name = getRosNodeName(ros::this_node::getName(), ros::this_node::getNamespace());
        rosComponentDescriptor.setName(node_name);
    }
	ROSComponent::~ROSComponent() {}

    int32_t ROSComponent::run() {
        setUp();

        while(rclcpp::ok()) {
            rclcpp::Rate loop_rate(rosComponentDescriptor.getFreq());
            rclcpp::spin_some(node);
            body();
            loop_rate.sleep();
        }

        tearDown();
        return 0;
    }

    std::string ROSComponent::getRosNodeName(const std::string& node_name, const std::string& node_namespace) {
        std::string ros_node_name = node_name;

        RCLCPP_DEBUG(rclcpp::get_logger("Archlib"), "%s", ros_node_name.c_str());
        size_t pos = ros_node_name.find(node_namespace + '/');

        if (pos == std::string::npos)
        {
            pos = ros_node_name.find(node_namespace);
            ros_node_name.replace(pos, node_namespace.length(), "/");
            return ros_node_name;
        }

        // RCLCPP_INFO(rclcpp::get_logger("Archlib"), "namespace %s", node_namespace.c_str());
        ros_node_name.replace(pos, node_namespace.length() + 1, "/");

        return ros_node_name;
    }
}
