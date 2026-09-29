#include "action_manager.hpp"
#include "action_service_traits.hpp"

using namespace action_manager;

class ActionManagerNode : public rclcpp::Node {

public:

    ActionManagerNode() : Node("action_manager"){
        const auto& mode = declare_parameter<std::string>("mode");
        const float update_interval = declare_parameter("update_interval",0.1f);
        if(mode == "goto"){
            action_ = std::make_unique<ActionManager<GotoAction, GotoSrv>>(*this, update_interval);
        }
        else if(mode == "path"){
            action_ = std::make_unique<ActionManager<PathAction, PathSrv>>(*this, update_interval);
        }
        else if(mode == "reference"){
            action_ = std::make_unique<ActionManager<ReferenceAction, ReferenceSrv>>(*this, update_interval);
        }else{
            RCLCPP_ERROR(get_logger(), "Unknown mode: %s", mode.data());
        }
    }

private:
    std::unique_ptr<IActionManager> action_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_manager = std::make_shared<ActionManagerNode>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

  executor.add_node(action_manager);
  executor.spin(); 
  
  rclcpp::shutdown();
  return 0;
}

