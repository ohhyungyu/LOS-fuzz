FUZZED_TARGET_DIR= # build/client_service_example
FUZZED_TARGET= # generated_fuzzer
HEAD_DIR= # /opt/ros_ws/src/client_service_example
RESULT_DIR=/root/RESULT/$FUZZED_TARGET

echo core | sudo tee /proc/sys/kernel/core_pattern

source $HEAD_DIR/install/setup.bash
# systemctl start fuzzing_hour_cvg_check.timer
systemctl start shm_deleter.timer

afl-fuzz \
	-i \
	$RESULT_DIR/inputs/ \
	-o \
	$RESULT_DIR/outputs/ \
	-m none \
	-t 1000+ \
	# -p mmopt CHOOSE THE POWER SCHEDULE
	-L 0 \
	-- \
	$HEAD_DIR/$FUZZED_TARGET_DIR/$FUZZED_TARGET

systemctl stop shm_deleter.timer