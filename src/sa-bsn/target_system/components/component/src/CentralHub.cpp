#include "component/CentralHub.hpp"

#include <iostream>

CentralHub::CentralHub(int &argc, char **argv, const std::string &name, const bool &active, const bsn::resource::Battery &battery) : Component(argc, argv, name), active(active), max_size(20), total_buffer_size(0), buffer_size({0,0,0,0,0,0}), battery(battery), data_buffer({{0},{0},{0},{0},{0},{0}}) {}

CentralHub::~CentralHub() {}

int32_t CentralHub::run() {
	setUp();

    rclcpp::Node nh;
    auto thermometerSub = nh.subscribe("thermometer_data", 10, &CentralHub::collect, this);
    auto oximeterSub = nh.subscribe("oximeter_data", 10, &CentralHub::collect, this);
    auto ecgSub = nh.subscribe("ecg_data", 10, &CentralHub::collect, this);
    auto abpsSub = nh.subscribe("abps_data", 10, &CentralHub::collect, this);
    auto abpdSub = nh.subscribe("abpd_data", 10, &CentralHub::collect, this);
    auto glucosemeterSub = nh.subscribe("glucosemeter_data", 10, &CentralHub::collect, this);
    auto reconfigSub = nh.subscribe("reconfigure_"+ros::this_node::getName(), 10, &CentralHub::reconfigure, this);

    while(rclcpp::ok()) {
        rclcpp::Rate loop_rate(rosComponentDescriptor.getFreq());

        try {
            body();
        } catch (const std::exception& e) {
            sendStatus("fail");
        }
        loop_rate.sleep();
    }

    return 0;
}

void CentralHub::body() {
    rclcpp::spin_some(node); //calls collect() if there's data in the topics

    if (!isActive() && battery.getCurrentLevel() > 90){
        turnOn();
    } else if (isActive() && battery.getCurrentLevel() < 2){
        turnOff();        
    }
    
    if(isActive()) {
        if(total_buffer_size > 0){
            apply_noise();
            process();
            transfer();
            sendStatus("success");
        }
    } else {
        recharge();
        throw std::domain_error("out of charge");
    }
}

void CentralHub::apply_noise() {}


void CentralHub::reconfigure(const archlib::AdaptationCommand::ConstPtr& msg) {
    std::string action = msg->action.c_str();

    std::vector<std::string> pairs = bsn::utils::split(action, ',');

    for (std::vector<std::string>::iterator it = pairs.begin(); it != pairs.end(); ++it){
        std::vector<std::string> param = bsn::utils::split(action, '=');

        // Why does replicate_collect updates frequency?
        /*if(param[0]=="replicate_collect"){
            rosComponentDescriptor.setFreq(rosComponentDescriptor.getFreq()+stoi(param[1]));
        }*/
        if(param[0]=="freq"){
            double new_freq = stod(param[1]);
            rosComponentDescriptor.setFreq(new_freq);
        }
    }
}

bool CentralHub::isActive() {
    return active;
}

void CentralHub::turnOn() {
    active = true;
    activate();
}

void CentralHub::turnOff() {
    active = false;
    deactivate();
}

/*  battery will always recover in 20seconds
    *
    *  b/s = 100% / 20 seconds = 5 %/s 
    *      => recovers 5% battery per second
    *  if we divide by the execution frequency
    *  we get the amount of battery we need to
    *  recover per execution cycle to achieve the
    *  5 %/s battery recovery rate
    */
void CentralHub::recharge() {
    if(battery.getCurrentLevel() <= 100) 
        battery.generate((100.0/20.0)/rosComponentDescriptor.getFreq());
}