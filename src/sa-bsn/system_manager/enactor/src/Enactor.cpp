#include "enactor/Enactor.hpp"

Enactor::Enactor(int &argc, char **argv, std::string name) : ROSComponent(argc, argv, name), cycles(0), stability_margin(0.02) {}

Enactor::~Enactor() {}

void Enactor::tearDown() {}

void Enactor::receiveAdaptationParameter() {
    rclcpp::Node client_handler;
    ros::ServiceClient client_module;

    client_module = client_handler.serviceClient<archlib::EngineRequest>("EngineRequest");
    archlib::EngineRequest adapt_srv;
    
    if(!client_module.call(adapt_srv)) {
        RCLCPP_ERROR(rclcpp::get_logger("Enactor"), "Failed to connect to Strategy Manager node.");
        return;
    }

    adaptation_parameter = adapt_srv.response.content;

    if(adaptation_parameter != "reliability" && adaptation_parameter != "cost") {
        RCLCPP_ERROR(rclcpp::get_logger("Enactor"), "Invalid adaptation parameter received.");
        return;
    }
}

void Enactor::receiveStatus() {
    rclcpp::Node client_handler;
    ros::ServiceClient client_module;

    client_module = client_handler.serviceClient<archlib::DataAccessRequest>("DataAccessRequest");
    archlib::DataAccessRequest r_srv;
    r_srv.request.name = ros::this_node::getName();
    if(adaptation_parameter == "reliability") {
        r_srv.request.query = "all:reliability:";
    } else {
        r_srv.request.query = "all:cost:";
    }

    if (!client_module.call(r_srv)) {
        RCLCPP_ERROR(rclcpp::get_logger("Enactor"), "Failed to connect to data access node.");
        return;
    }
    
    std::string ans = r_srv.response.content;
    if (ans == "") {
        RCLCPP_ERROR(rclcpp::get_logger("Enactor"), "Received empty answer when asked for status.");
    }

    std::vector<std::string> pairs = bsn::utils::split(ans, ';');

    for (auto s : pairs) {
        std::vector<std::string> pair = bsn::utils::split(s, ':');
        std::string component = pair[0];
        std::string content = pair[1];

        std::vector<std::string> values = bsn::utils::split(content, ',');

        if(adaptation_parameter == "reliability") {
            r_curr[component] = stod(values[values.size() - 1]);
            apply_reli_strategy(component);
        } else {
            c_curr[component] = stod(values[values.size() - 1]);
            apply_cost_strategy(component);
        }
    }
}

void Enactor::receiveStrategy(const archlib::Strategy::ConstPtr& msg) {     

    std::vector<std::string> refs = bsn::utils::split(msg->content, ';');

    for(std::vector<std::string>::iterator ref = refs.begin(); ref != refs.end(); ref++){
        std::vector<std::string> pair = bsn::utils::split(*ref, ':'); 
        if(adaptation_parameter == "reliability") {
            r_ref[pair[0]] = stod(pair[1]);
        } else {
            c_ref[pair[0]] = stod(pair[1]);
        }
    }
}

void Enactor::body(){
    rclcpp::Node n;

    auto subs_event = n.subscribe("event", 1000, &Enactor::receiveEvent, this);
    auto subs_strategy = n.subscribe("strategy", 1000, &Enactor::receiveStrategy, this);

    rclcpp::Rate loop_rate(rosComponentDescriptor.getFreq());
    while(rclcpp::ok()){
        if(cycles <= 60*rosComponentDescriptor.getFreq()) ++cycles;
        receiveStatus();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}

void Enactor::print() {
    /*
    for(std::vector<std::string>::iterator it = connected.begin(); it != connected.end(); ++it) {
        RCLCPP_DEBUG(rclcpp::get_logger("Enactor"), "****************************************");
        RCLCPP_DEBUG(rclcpp::get_logger("Enactor"), "%s - - invocations: [",*it);
        for(std::deque<int>::iterator itt = invocations[*it].begin(); itt != invocations[*it].end(); ++itt) RCLCPP_DEBUG(rclcpp::get_logger("Enactor"), *it%st," ";
        RCLCPP_DEBUG(rclcpp::get_logger("Enactor"), "]");
        RCLCPP_DEBUG(rclcpp::get_logger("Enactor"), "  - r curr: %s",r_curr[*it]);
        RCLCPP_DEBUG(rclcpp::get_logger("Enactor"), "  - buffer size: %s",replicate_task[*it]);
        RCLCPP_DEBUG(rclcpp::get_logger("Enactor"), "****************************************");
    }
    */
}