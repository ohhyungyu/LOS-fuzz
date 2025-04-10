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

{{ IMPORTES }}

{{ FUZZING_API }}


static void kill_pid(const pid_t& pid)
{
    std::cout << "Time is up. Good job! Killing parent." << std::endl;
    kill(pid, SIGRTMAX);
    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
}

static void treat_timeout_signal(int signum)
{
    if (signum == SIGRTMAX) {
        std::cout << "It is time to finish!" << std::endl;
        rclcpp::shutdown();
        exit(EXIT_SUCCESS);
    }
}




{{ REQUEST_CODES }}


int main(int argc_fuzz, char *argv_fuzz[])
{
	pid_t parent_pid = getpid();
	pid_t pid = fork();

	if (pid < 0) {
		std::cout << "Something crashed" << std::endl;
		exit(EXIT_FAILURE);
	} else if (pid == 0) {
        // ------------------------- Have to Change
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
    return 0;
}
