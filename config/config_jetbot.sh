#!/bin/bash
show_help() {
    echo " "
    echo "  usage: Configure the jetbot"
    echo "         THIS SHOULD ALWAYS BE SOURCED FROM THE PARENT DIRECTORY (sycabot_ros)"
    echo "          ./configure_jetbot.sh --file LAUNCH_FILE_NAME"
    echo ""
    echo ""
    echo ""
    echo "  args :"
    echo ""
    echo "      --help          Show this help"
    echo ""
    echo "      --file Launch file you want to execute at boot. keys : motors, init, motorsRL, boot"
    echo "             default : init"
    echo ""
    echo "      --id Id of the jetbot being configured. keys : integer"
    echo "             default : 1"
    echo ""
    echo "      --install Install necessary packages (should be used at first initialization)"
    echo ""
    echo ""
    echo ""

}
die() {
    printf '%s\n' "$1"
    show_help
}

LAUNCH_FILE_NAME="init"
ID="1"

while :; do
    case $1 in
        -h|-\?|--help)
            show_help    # Display a usage synopsis.
            exit
            ;;
        --file)
            if [ "$2" == "motors" ];then
                LAUNCH_FILE_NAME="$2"
                sudo sed -i -r "s/from sycabot_base.motors[A-Z]+/from sycabot_base.motors/" sycabot_base/sycabot_base/motors_waveshare.py
                cat sycabot_base/sycabot_base/motors_waveshare.py
                shift
            elif [ "$2" == "init" ];then
                LAUNCH_FILE_NAME="$2"
                
            elif [ "$2" == "boot" ];then
                sudo sed -i -r "s/from sycabot_base.motors[A-Z]+/from sycabot_base.motorsRL/" sycabot_base/sycabot_base/motors_waveshare.py
                cat sycabot_base/sycabot_base/motors_waveshare.py
                LAUNCH_FILE_NAME="$2"
                shift
            elif [ "$2" == "motorsRL" ];then
                LAUNCH_FILE_NAME="motors"
                sudo sed -i -r "s/from sycabot_base.motors[A-Z]+/from sycabot_base.motorsRL/" sycabot_base/sycabot_base/motors_waveshare.py
                cat sycabot_base/sycabot_base/motors_waveshare.py
                shift
            else
                die 'ERROR: "--pkg" wrong input argument.'
            fi
            ;;
        --id)
            if [ "$2" ];then
                ID="$2"
                echo "$ID"
                shift
            else
                die 'ERROR: "--id" requires a non-empty option argument.'
            fi
            ;;
        --install)
            echo 'installing adafruit ...'
            sudo -H pip3 install Adafruit-MotorHAT Adafruit-SSD1306 pyserial sparkfun-qwiic --verbose

            echo 'install matplotlib ...'
            # https://forums.developer.nvidia.com/t/jetson-nano-how-can-install-matplotlib/75132/7
            sudo cp /etc/apt/sources.list /etc/apt/sources.list~
            sudo sed -Ei 's/^# deb-src /deb-src /' /etc/apt/sources.list
            sudo apt-get update
            cd ~/.local/lib/python3.6/site-packages
            git clone -b v3.3.4 --depth 1 https://github.com/matplotlib/matplotlib.git
            cd ~/.local/lib/python3.6/site-packages/matplotlib
            sudo apt-get build-dep python3-matplotlib -y
            # pip3 install . -v
            sudo mv /etc/apt/sources.list~ /etc/apt/sources.list
            sudo apt-get update

            echo 'installing control ...'
            sudo pip3 install control --verbose

            echo 'Upgrade pip ...'
            python3 -m pip install --upgrade pip

            echo 'installing acados ...'
            git clone https://github.com/acados/acados.git
            cd acados
            git submodule update --recursive --init
            mkdir -p build
            cd build
            cmake -DACADOS_WITH_QPOASES=ON ..
            # add more optional arguments e.g. -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
            make install -j4
            cd
            ACADOS_ROOT="/home/jetbot/.local/lib/python3.6/site-packages/acados"
            pip3 install -e "${ACADOS_ROOT}/interfaces/acados_template"

            echo 'Updating tera_renderer ...'
            cd /home/jetbot
            mkdir tmp
            cd tmp
            git clone https://github.com/acados/tera_renderer
            cd tera_renderer
            sudo apt-get -y install cargo
            cargo build --verbose --release
            cp target/release/t_renderer $ACADOS_ROOT/bin/


            shift
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

#Create the good directory
cd ~/sycabot_ros
cp -r ./. ../syca_ws/

# Copy service file and make systemctl recgonize it
# Set the boot file
cd ../syca_ws
sudo sed -i -r "s/ros2 launch sycabot_launch [a-z]+/ros2 launch sycabot_launch ${LAUNCH_FILE_NAME}/" config/boot_init.sh
echo ""
echo ""
cat config/boot_init.sh
echo ""
echo ""
echo 'Copying service and boot file...'
sudo cp config/robot_boot.service /lib/systemd/system/
sudo cp config/boot_init.sh /usr/local/bin/
echo 'chown /usr/local/bin/boot_init.sh'
sudo chown root:root /usr/local/bin/boot_init.sh
echo 'chmod 755 /usr/local/bin/boot_init.sh'
sudo chmod 755 /usr/local/bin/boot_init.sh
echo 'daemon reload'
sudo systemctl daemon-reload

# Change SYCABOT_ID number : https://www.geeksforgeeks.org/sed-command-in-linux-unix-with-examples/
cd ../syca_ws/sycabot_launch/launch
sudo sed -i "s/SYCABOT_ID = ./SYCABOT_ID = ${ID}/" init.launch.py
sudo sed -i "s/SYCABOT_ID = ./SYCABOT_ID = ${ID}/" motors.launch.py
sudo sed -i "s/SYCABOT_ID = ./SYCABOT_ID = ${ID}/" boot.launch.py
echo ""
echo ""
cat init.launch.py
echo ""
echo ""
cat motors.launch.py
echo ""
echo ""
cat boot.launch.py
echo ""
echo ""
cd ~/syca_ws