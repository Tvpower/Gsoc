#include "my_rfid_demo/dummy_sensor.hpp"

#include <string>
#include <gz/plugin/Register.hh>
#include <gz/msgs//stringmsg.pb.h>

using namespace my_rfid_demo;

DummySensor::DummySensor() {

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

    //create message
    gz::msgs::StringMsg msg;
    msg.set_data("Hello world");

    //publish it!
    this->publisher.Publish(msg);
}

GZ_ADD_PLUGIN(
    DummySensor,
    gz::sim::System,
    DummySensor::ISystemConfigure,
    DummySensor::ISystemUpdate)


