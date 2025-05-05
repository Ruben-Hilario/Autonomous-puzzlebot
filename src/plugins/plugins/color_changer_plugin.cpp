#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/material.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <thread>
#include <mutex>

using namespace gz;
using namespace gz::sim;

class ColorChangerPlugin : public System,
                           public ISystemConfigure,
                           public ISystemPreUpdate
{
public:
  void Configure(const Entity &entity,
                 const std::shared_ptr<const sdf::Element> &,
                 EntityComponentManager &,
                 EventManager &) override
  {    
    this->model = Model(entity);
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    this->node = std::make_shared<rclcpp::Node>("color_changer_plugin");
    this->rosSub = this->node->create_subscription<std_msgs::msg::String>(
        "/color_command", 10,
        std::bind(&ColorChangerPlugin::OnColorMsg, this, std::placeholders::_1));
    this->rosThread = std::thread([this]() { rclcpp::spin(this->node); });
  }

  void OnColorMsg(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->targetColor = msg->data;
    this->colorChanged = true;
  }

  void PreUpdate(const UpdateInfo &, EntityComponentManager &) override
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->colorChanged)
      return;

    msgs::Material matMsg;

    if (this->targetColor == "red") {
      matMsg.mutable_diffuse()->set_r(1.0f);
      matMsg.mutable_diffuse()->set_g(0.0f);
      matMsg.mutable_diffuse()->set_b(0.0f);
    } else if (this->targetColor == "green") {
      matMsg.mutable_diffuse()->set_r(0.0f);
      matMsg.mutable_diffuse()->set_g(1.0f);
      matMsg.mutable_diffuse()->set_b(0.0f);
    } else if (this->targetColor == "yellow") {
      matMsg.mutable_diffuse()->set_r(1.0f);
      matMsg.mutable_diffuse()->set_g(1.0f);
      matMsg.mutable_diffuse()->set_b(0.0f);
    }

    gz::transport::Node gzNode;
    gzNode.Request("/world/default/set_material", matMsg);

    this->colorChanged = false;
  }

private:
  Model model{kNullEntity};
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rosSub;
  std::thread rosThread;
  std::mutex mutex;

  std::string targetColor{"red"};
  bool colorChanged{false};
};
#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(ColorChangerPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)