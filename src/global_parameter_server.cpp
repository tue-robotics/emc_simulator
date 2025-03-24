#include "rclcpp/rclcpp.hpp"
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::cout << "parameter server intialized!" << std::endl;
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("global_parameter_server", options);
    std::cout << "parameter server running!" << std::endl;
    rclcpp::spin(node);
    std::cout << "parameter server shutting down!" << std::endl;
    rclcpp::shutdown();
    return 0;
}