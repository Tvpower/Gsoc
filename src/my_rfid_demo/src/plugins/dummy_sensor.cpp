#include "my_rfid_demo/dummy_sensor.hpp"

#include <string>
#include <chrono>
#include <gz/plugin/Register.hh>
#include <gz/msgs//stringmsg.pb.h>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

using namespace my_rfid_demo;

DummySensor::DummySensor() : last_publish_time(0) {

}

DummySensor::~DummySensor() {

}

void DummySensor::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    //get the topics name from SDF
    this->topic_name = "/dummy_sensor";
    if (_sdf->HasElement("topic")) {
        this->topic_name = _sdf->Get<std::string>("topic");
    }


    //store the entity
    this->entity = _entity;

    //advertise the topic
    this->publisher = this->node.Advertise<gz::msgs::StringMsg>(this->topic_name);
}

void DummySensor::Update(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{

    //only publish on simulation iterations (not the actual real time updates)
    if (_info.paused) {
        return;
    }

    //limit publishing time
    auto current_time = _info.simTime.count();
    if (current_time - this->last_publish_time < 10000000000) {
        return;
    }
    this->last_publish_time = current_time;


    //save models in the world
    std::vector<std::string> tags;


    //simulate detection by checking nearby models
    if (current_time % 5000000000 < 2000000000) {
        tags.emplace_back("pallet1:RFID:12345");

    } else if (current_time % 7000000000 < 3000000000) {
        tags.emplace_back("box2:RFID:67890");
    }

    //create message
    gz::msgs::StringMsg msg;

    if (tags.empty()) {
        msg.set_data("Hello world - No RFID tags detected");
    } else {
        std::string data = "Hello world - Detected tags: ";
        for (const auto& tag : tags) {
            data += tag + ", ";
        }
        msg.set_data(data);
    }

    //publish it!
    this->publisher.Publish(msg);
}



GZ_ADD_PLUGIN(
    DummySensor,
    gz::sim::System,
    DummySensor::ISystemConfigure,
    DummySensor::ISystemUpdate)


