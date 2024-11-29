#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_srvs/srv/empty.hpp"
#include "service/add_two_ints_client.hpp"

class SimpleNode : public rclcpp::Node
{
public:
    explicit SimpleNode(const rclcpp::NodeOptions & options)
    : Node("simple_node", options)
    {
        auto simple_callback = 
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   const std::shared_ptr<std_srvs::srv::Empty::Response> response) -> void
            {
                client_ = std::make_unique<ClientNode>(shared_from_this());
                RCLCPP_INFO(this->get_logger(), "Execute simple callback.");
                client_->execute();
                RCLCPP_INFO(this->get_logger(), "Finished simple callback.");
            };

        add_two_ints_srv_ = this->create_service<std_srvs::srv::Empty>("/simple_node/add_two_ints", simple_callback);
    }

private:
    std::unique_ptr<ClientNode> client_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr add_two_ints_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr initialize_srv_;
};

// Register the node as a component
RCLCPP_COMPONENTS_REGISTER_NODE(SimpleNode)
