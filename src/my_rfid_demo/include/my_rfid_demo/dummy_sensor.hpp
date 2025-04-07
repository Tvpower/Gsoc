#ifndef MY_RFID_DEMO_DUMMY_SENSOR_HPP_
#define MY_RFID_DEMO_DUMMY_SENSOR_HPP_

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace my_rfid_demo {
    class DummySensor:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemUpdate
    {
    public: DummySensor();
    public: ~DummySensor() override;

    public: void Configure(
            const gz::sim::Entity &_entity,
            const std::shared_ptr<const sdf::Element> &_sdf,
            gz::sim::EntityComponentManager &_ecm,
            gz::sim::EventManager &_eventMgr) override;

    public: void Update(
            const gz::sim::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm) override;

    private: gz::transport::Node node;
    private: std::string topic_name;
    private: gz::transport::Node::Publisher publisher;
    };

}
#endif  // MY_RFID_DEMO_DUMMY_SENSOR_HPP_
