ARG ROSDIST=foxy
FROM ros:$ROSDIST
ENV ROSDIST=foxy

# for installation apt-utils, .., etc.
ENV DEBIAN_FRONTEND=noninteractive

RUN /bin/bash -c "source /opt/ros/${ROSDIST}/setup.bash"

# TODO: remove (use rosdep install instead)
RUN apt-get update && \
	apt-get install -y apt-utils 2>&1 | grep -v "debconf: delaying package configuration, since apt-utils is not installed" && \
    apt-get install -y --no-install-recommends ros-foxy-example-interfaces && \
    apt-get install -y vim && \
    apt-get install -y python3-pip

# Installation for SYSTEMCTL
RUN apt update
RUN apt-get install -qq -y init systemd

# Installation for AFL++
RUN apt update && apt-get upgrade -y
RUN apt-get install -y build-essential python3-dev \
	automake cmake git flex bison libglib2.0-dev \
	libpixman-1-dev python3-setuptools cargo libgtk-3-dev
# try to install llvm 14 and install the distro default if that fails
RUN apt-get install -y lld-14 llvm-14 llvm-14-dev clang-14 ||\
	apt-get install -y lld llvm llvm-dev clang

RUN apt-get install -y \
	gcc-$(gcc --version|head -n1|sed 's/\..*//'|sed 's/.* //')-plugin-dev \
	libstdc++-$(gcc --version|head -n1|sed 's/\..*//'|sed 's/.* //')-dev

RUN apt-get install -y ninja-build
RUN apt-get install -y cpio libcapstone-dev
RUN apt-get install -y wget curl

# Download lib-dev
RUN apt-get install -y build-essential \
		libboost-system-dev \
		libboost-thread-dev \
		libboost-program-options-dev \
		libboost-test-dev

# Installation AFL++
WORKDIR /root
RUN git clone "https://github.com/AFLplusplus/AFLplusplus"
RUN /bin/bash -c "make -C AFLplusplus all"
RUN /bin/bash -c "make -C AFLplusplus install"


ENV ROS_WS=/opt/ros_ws
WORKDIR $ROS_WS
RUN mkdir -pv $ROS_WS/src

# Copy packages
COPY ./FUZZ_TARGET src/

# ENV update & Download the required package
RUN /bin/bash -c "apt-get update && rosdep install -i --from-path src --rosdistro=${ROSDIST} -y"

# pre-build the Package
RUN /bin/bash -c "source /opt/ros/${ROSDIST}/setup.bash && colcon build"

# Copy the python package, install it and erase the source files
# TODO: move to pip; this is fragile
COPY ./ros2_automatic_fuzzer /temporary/ros2_automatic_fuzzer/
RUN pip3 install -e /temporary/ros2_automatic_fuzzer/

# configuration setting
COPY ./FUZZ_UTIL/vimrc /root/.vimrc
COPY ./FUZZ_UTIL/SETUP /root/.SETUP
RUN cat /root/.SETUP 1>> /root/.bashrc

# fuzz setup
ENV FUZZ_UTIL_DIR="/root/RESULT"

COPY ./FUZZ_UTIL/fuzz_start.sh /root/fuzz_start.sh

# CVG, crash check file COPY
RUN mkdir -pv $FUZZ_UTIL_DIR/CVG_CHECK

COPY ./FUZZ_UTIL/cvg_check/cvg_check.c $FUZZ_UTIL_DIR/CVG_CHECK/
RUN /bin/bash -c "gcc -o \
	/root/RESULT/CVG_CHECK/cvg_check \
	/root/RESULT/CVG_CHECK/cvg_check.c"

COPY ./FUZZ_UTIL/cvg_check/fuzzing_hour_cvg_check.timer /lib/systemd/system/
COPY ./FUZZ_UTIL/cvg_check/fuzzing_hour_cvg_check.service /lib/systemd/system/

# SHM file deleter COPY
RUN mkdir -pv $FUZZ_UTIL_DIR/SHM_DELETER
COPY ./FUZZ_UTIL/shm_deleter/shm_deleter.sh $FUZZ_UTIL_DIR/SHM_DELETER/

COPY ./FUZZ_UTIL/shm_deleter/shm_deleter.service /lib/systemd/system/
COPY ./FUZZ_UTIL/shm_deleter/shm_deleter.timer /lib/systemd/system/

# FUZZ UTIL : DIR Generator
COPY ./FUZZ_UTIL/FUZZ_DIR_GEN.sh $FUZZ_UTIL_DIR/FUZZ_DIR_GEN.sh
