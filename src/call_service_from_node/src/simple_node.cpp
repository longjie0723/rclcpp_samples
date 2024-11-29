#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_srvs/srv/empty.hpp"
#include "call_service_from_node/add_two_ints_client.hpp"

class SimpleNode : public rclcpp::Node
{
public:
    explicit SimpleNode(const rclcpp::NodeOptions & options)
    : Node("simple_node", options)
    {
        srv_ = this->create_service<std_srvs::srv::Empty>(
            "/simple_node/add_two_ints", 
            std::bind(&SimpleNode::simple_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    void initialize()
    {
        //client_ = std::make_unique<ClientNode>(shared_from_this());
        client_ = std::make_shared<ClientNode>(shared_from_this());
    }

    void simple_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   const std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Execute simple node callback.");
        client_->execute();
        RCLCPP_INFO(this->get_logger(), "Finished simple node callback.");
    }

private:
    //std::unique_ptr<ClientNode> client_;
    std::shared_ptr<ClientNode> client_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_;
};

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    std::shared_ptr<SimpleNode> node = std::make_shared<SimpleNode>(options);
    node->initialize();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
}

// Register the node as a component
// RCLCPP_COMPONENTS_REGISTER_NODE(SimpleNode)
