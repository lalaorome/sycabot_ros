#!/bin/bash

show_help() {
    echo " "
    echo "  usage: Source ROS2 + eventually set fast dds server ip and source local setup"
    echo "         THIS SHOULD ALWAYS BE SOURCED FROM THE PARENT DIRECTORY (sycabot_ros)"
    echo "          ./source_ros.sh --pkg PKG_NAME"
    echo "                          --srv IP_ID"
    echo ""
    echo ""
    echo "  args :"
    echo ""
    echo "      --help          Show this help"
    echo ""
    echo "      --pkg Source local setup keys."
    echo ""
    echo "      --srv Id of the ip adress of the fast dds server."
    echo "            the id is the x in 192.168.1.x"
    echo ""
    echo ""

}
die() {
    printf '%s\n' "$1"
    show_help
}

#Source ROS2 foxy :
source /opt/ros/foxy/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

if [ $USER == "jetbot" ];then
    echo "jetbot user detected ..."
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/home/jetbot/.local/lib/python3.6/site-packages/acados/lib"
    export ACADOS_SOURCE_DIR="/home/jetbot/.local/lib/python3.6/site-packages/acados"
else
    echo "central user detected ..."
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/home/sycamore/tmp/acados/lib"
    export ACADOS_SOURCE_DIR="/home/sycamore/tmp/acados"
fi 

while :; do
    case $1 in
        -h|-\?|--help)
            show_help    # Display a usage synopsis.
            ;;
        --pkg)
            echo "sourcing central_pc..."
            source $PWD/install/local_setup.bash
            ;;
        --srv)
            if [ "$2" ];then
                echo "setting discovery server ip..."
                echo $2
                export ROS_DISCOVERY_SERVER="192.168.1.$2:11811"
                shift
            else
                die 'ERROR: "--srv" requires a non-empty option argument.'
            fi
            ;;
        --)              # End of all options.
            shift
            break
            ;;
        -?*)
            printf 'WARN: Unknown option (ignored): %s\n' "$1" >&2
            ;;
        *)               # Default case: No more options, so break out of the loop.
            break
    esac
    shift
done



