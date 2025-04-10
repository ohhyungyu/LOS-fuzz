#include <memory>
#include <cstdint>
#include <cstdlib>
#include <chrono>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <signal.h>
#include <bits/signum.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

{{ IMPORTS }}

{{ FUZZING_API }}

void {{ FUZZ_NAME }}(int argc, char const *const argv[]) 
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("{{ NODE_NAME }}");

    // 액션 클라이언트 생성
    auto action_client = rclcpp_action::create_client<{{ NODE_TYPE }}>(node, "{{ CLIENT_NAME }}");
  
    // 서버 대기
    if (!action_client->wait_for_action_server(std::chrono::milliseconds(500))) {
      RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
      return ;
    }
    // request code 로 들어가야 할 내용 : auto goal_msg = {{ NODE_TYPE }}::Goal();

    {{ REQUEST_CODE }}

    auto goal_future = action_client->async_send_goal(goal_msg);
    rclcpp::spin_until_future_complete(node, goal_future);
    
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node->get_logger(), "Goal was rejected");
      return ;
    }
    
    // 여기서 끝내면 goal은 성공적으로 서버에 전달된 것 ✔
    rclcpp::shutdown();
    return ;
    
}