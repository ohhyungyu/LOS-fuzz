BUILD_NAME="" # BUILD_NAME
BUILD_DIR=$PWD/../

if [ $BUILD_NAME -eq ""]
then
        exit
fi

cd $BUILD_DIR

echo "Choose To do
        1. build&run
        2. run
        3. kill&rm
        4. exec
        5. start
        6. stop"
read idx

if [ $idx -eq "1" ]
then
        docker build -t $BUILD_NAME .
        docker run --privileged=true -it -d --cap-add=SYS_PTRACE --name=$BUILD_NAME $BUILD_NAME /sbin/init
elif [ $idx -eq "2" ]
then
        docker run --privileged=true -it -d --cap-add=SYS_PTRACE --name=$BUILD_NAME $BUILD_NAME /sbin/init
elif [ $idx -eq "3" ]
then
        docker kill $BUILD_NAME
        docker rm $BUILD_NAME
elif [ $idx -eq "4" ]
then
        docker exec -it -u root $BUILD_NAME bash
elif [ $idx -eq "5" ]
then
        docker start $BUILD_NAME
elif [ $idx -eq "6" ]
then
        docker stop $BUILD_NAME
else
        echo "NOT VALID COMMAND"
fi

exit
