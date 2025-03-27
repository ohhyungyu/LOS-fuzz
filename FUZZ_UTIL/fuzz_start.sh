FUZZED_TARGET_DIR=build/nav2_amcl #build/nav2_bt_navigator # build/nav2_amcl
FUZZED_TARGET=amcl # bt_navigator # amcl
HEAD_DIR=/opt/ros_ws/src/nav2_amcl # nav2_bt_navigator # nav2_amcl
RESULT_DIR=/root/RESULT/$FUZZED_TARGET

if [ $CXX -ne "afl-g++" ]
then
	SETUP
fi

AFL_I_DONT_CARE_ABOUT_MISSING_CRASHES=1

source $HEAD_DIR/install/setup.bash
systemctl start fuzzing_hour_cvg_check.timer

afl-fuzz \
	-i \
	$RESULT_DIR/inputs/ \
	-o \
	$RESULT_DIR/outputs/ \
	-m none \
	-t 1000+ \
	-- \
	$HEAD_DIR/$FUZZED_TARGET_DIR/$FUZZED_TARGET
