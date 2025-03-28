/*
 * This is an automatically generated file. Do not modify.
 * 
 * This file contains the ros2_automatic_fuzzer implementation
 * for the `{{ FILE_NAME }}` publisher source file.
 */
#include <iostream>

#include <string>
#include <istream>
#include <ostream>
#include <iterator>
#include <chrono>
#include <functional>
#include <memory>
#include <unistd.h>
#include <signal.h>
#include <bits/signum.h>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

{{ IMPORTS }}

{{ FUZZING_API }}

/* 
{{ REQUEST_CODE }}
*/

class FuzzerPublisher : public rclcpp::Node
{
public:
	FuzzerPublisher(int parent_pid)
	: Node("fuzzer__publisher_"), parent_pid_(parent_pid)
	{
		RCLCPP_INFO(this->get_logger(), "Started publisher from fuzz_target");
		publisher_ = this->create_publisher<std_msgs::msg::String>("{{ CLIENT_NAME }}", 10);
		timerTimeout_ = this->create_wall_timer(
			10000ms, std::bind(&FuzzerPublisher::timer_timeout, this));
		timer_ = this->create_wall_timer(
			1ms, std::bind(&FuzzerPublisher::timer_callback, this));
	}

private:

	// Kill parent and ourselves if time is up 
	void timer_timeout()
	{
		RCLCPP_INFO(this->get_logger(), "Time is up. Good job! Killing parent.");
		kill(parent_pid_, SIGRTMAX);
		rclcpp::shutdown();
		exit(EXIT_SUCCESS);
	} 

	void timer_callback()
	{
		detect_crash();

		/* fuzzed topic instance as request*/
		auto request; // NEED TO CHANGE

       {{ REQUEST_CODE }}

		publisher_->publish(request);
	}

	void detect_crash()
	{
		// A crash happens if the parent id is different from the original one
		// or simply if there is no PID running. Note that kill 0 doesn't send any signal!
		if (getppid() != parent_pid_ || 0 != kill(parent_pid_, 0)) {
			RCLCPP_INFO(this->get_logger(), "The node under test has crashed.\n"
				"Last input `%s` may have produced a crash.\n"
				"Stopping fuzzer.\n", input_.c_str() );
			rclcpp::shutdown();
			exit(EXIT_SUCCESS);
		}
	}
 
	static std::string get_all_input()
	{
	  std::cin >> std::noskipws;
	  std::istream_iterator<char> it(std::cin);
	  std::istream_iterator<char> end;
	  std::string results(it, end);
	  results.pop_back();
	  return results;
	}

	// Private fields
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr timerTimeout_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	
	pid_t parent_pid_;
	std::string input_;
};

static void fuzz_target(int argc, char* argv[], pid_t parent_pid)
{
	std::cout << "Fuzzer started" << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FuzzerPublisher>(parent_pid));
	rclcpp::shutdown();
}


static void treat_timeout_signal(int signum)
{
	if (signum == SIGRTMAX) {
		std::cout << "It is time to finish!" << std::endl;
		rclcpp::shutdown();
		exit(EXIT_SUCCESS);
	}
}

int main(int argc_fuzz, char *argv_fuzz[])
{
	pid_t parent_pid = getpid();
	pid_t pid = fork();

	if (pid < 0) {
		std::cout << "Something crashed" << std::endl;
		exit(EXIT_FAILURE);
	} else if (pid == 0) {
		fuzz_target(argc_fuzz, argv_fuzz, parent_pid);
		exit(EXIT_FAILURE);
	}

	// Parent's code
	signal(SIGRTMAX, treat_timeout_signal);

	// Close standard input in the node
	close(0);

	// Continue normal system under test code
	std::cout << "Continuing normal code" << std::endl;
	PREVIOUS_main(argc_fuzz, argv_fuzz);
}