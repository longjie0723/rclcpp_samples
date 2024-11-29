#include "call_service_in_lifecycle_node/add_two_ints_client.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rclcpp_components/register_node_macro.hpp"


class SimpleLifecycle : public rclcpp_lifecycle::LifecycleNode
{
public:
    SimpleLifecycle(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("simple_lifecycle", options)
    {
        srv_ = this->create_service<std_srvs::srv::Empty>(
            "/simple_lifecycle/add_two_ints",
            std::bind(&SimpleLifecycle::handle_add_two_ints, this, std::placeholders::_1, std::placeholders::_2)
        );
    };

    void initialize()
    {
        client_ = std::make_shared<ClientNode>(shared_from_this());
    }

    void handle_add_two_ints(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        RCLCPP_INFO(get_logger(), "Execute /simple_lifecycle/add_two_ints.");
        client_->execute();
        RCLCPP_INFO(get_logger(), "Finished /simple_lifecycle/add_two_ints.");
    }

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    
    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring");
        client_->execute();
        return CallbackReturn::SUCCESS;
    }
    
    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating");
        return CallbackReturn::SUCCESS;
    }
    
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating");
        return CallbackReturn::SUCCESS;
    }
    
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up");
        return CallbackReturn::SUCCESS;
    }
    
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Shutting down");
        return CallbackReturn::SUCCESS;
    }
    
    CallbackReturn on_error(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Erroring");
        return CallbackReturn::SUCCESS;
    };
private:
    std::shared_ptr<ClientNode> client_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_;
    
};

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;
    std::shared_ptr<SimpleLifecycle> node = std::make_shared<SimpleLifecycle>(options);
    node->initialize();
    executor.add_node(node->get_node_base_interface());
    executor.spin();
}

// RCLCPP_COMPONENTS_REGISTER_NODE(SimpleLifecycle)
