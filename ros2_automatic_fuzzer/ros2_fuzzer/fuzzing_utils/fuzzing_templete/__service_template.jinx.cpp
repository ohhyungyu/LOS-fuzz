void {{ FUZZ_NAME }}(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    // Create node
    auto node = rclcpp::Node::make_shared("{{ NODE_NAME }}");

    rclcpp::Client<{{ NODE_TYPE }}>::SharedPtr client = 
        node->create_client<{{ NODE_TYPE }}>("{{ CLIENT_NAME }}");

    auto request = std::make_shared<{{ NODE_TYPE }}::Request>();

    /* START SINGLE ITERATION */

{{ REQUEST_CODE }}

    while (!client->wait_for_service(std::chrono::milliseconds(500))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("automatic_fuzzing_logger"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("automatic_fuzzing_logger"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        //RCLCPP_INFO(rclcpp::get_logger("automatic_fuzzing_logger"), "Sum: %ld", result.get()->sum);
        RCLCPP_INFO(rclcpp::get_logger("automatic_fuzzing_logger"), "Received response!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("automatic_fuzzing_logger"), "Failed to call service");
    }

    /* FINISH SINGLE ITERATION */

    rclcpp::shutdown();
    return ;
}