void {{ FUZZ_NAME }}(int argc, char const *const argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("{{ NODE_NAME }}");

  // Generate Action Client
  auto action_client = rclcpp_action::create_client<{{ NODE_TYPE }}>(node, "{{ CLIENT_NAME }}");

  // Input API
  auto request = {{ NODE_TYPE }}::Goal();
  
{{ REQUEST_CODE }}


  // wating Server for 500ms
  if (!action_client->wait_for_action_server(std::chrono::milliseconds(500))) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    return ;
  }

  auto result = action_client->async_send_goal(request);
  rclcpp::spin_until_future_complete(node, result);
  
  auto goal_handle = result.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected");
    return ;
  }
  
  // end
  rclcpp::shutdown();
  return ;
}