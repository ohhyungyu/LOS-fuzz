void {{ FUZZ_NAME }}(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("{{ NODE_NAME }}");

    auto publisher = node->create_publisher<{{ NODE_TYPE }}>(
    "{{ CLIENT_NAME }}", 10);

    {{ NODE_TYPE }} request;

{{ REQUEST_CODE }}

    publisher->publish(request);
    rclcpp::shutdown();
    return ;
}