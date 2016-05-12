#!/bin/bash

function configure_user {

USERNAME=$1
USERHOME=/home/$USERNAME
PASSWORD=`echo -n $USERNAME$SALT | md5sum | head --bytes=8`

if ! id -u $1 >/dev/null 2>&1; then
  adduser --disabled-password --gecos "" $USERNAME
  echo "$USERNAME:$PASSWORD" | chpasswd
fi

adduser $USERNAME turtlebot 

pushd $USERHOME

ROS_WORKSPACE=$USERHOME/ROS

mkdir -p $USERHOME/.ros

chown -R $USERNAME:$USERNAME $USERHOME/.ros || true

sudo -u $USERNAME -H rosdep update || true

# environment setup
if ! grep -Fxq "source /opt/ros/$ROS_DIST/setup.bash" $USERHOME/.bashrc; then
  echo "source /opt/ros/$ROS_DIST/setup.bash" >> $USERHOME/.bashrc
fi

if [ ! -d $ROS_WORKSPACE ]; then
	mkdir $ROS_WORKSPACE
	chown $USERNAME:$USERNAME $ROS_WORKSPACE
fi

if [ ! -d $ROS_WORKSPACE/src ]; then
	mkdir $ROS_WORKSPACE/src
	chown $USERNAME:$USERNAME $ROS_WORKSPACE/src
	pushd $ROS_WORKSPACE/src
	sudo -u $USERNAME bash -c "source /opt/ros/$ROS_DIST/setup.bash; /opt/ros/$ROS_DIST/bin/catkin_init_workspace"
	popd
	pushd $ROS_WORKSPACE
	sudo -u $USERNAME bash -c "source /opt/ros/$ROS_DIST/setup.bash; /opt/ros/$ROS_DIST/bin/catkin_make"
	popd
fi

if ! grep -Fxq "# AUTOGENERATED ROS CONFIG !!!" $USERHOME/.bashrc; then
BASH_CONFIG="
# AUTOGENERATED ROS CONFIG !!!
source /usr/local/bin/roshost
alias roshost=\". /usr/local/bin/roshost\"
export ROS_WORKSPACE="$ROS_WORKSPACE"
export ROBOT=\`hostname\`
export TURTLEBOT_BATTERY=\"/sys/class/power_supply/BAT0\"
export TURTLEBOT_BASE=\"roomba\"
export TURTLEBOT_STACKS=\"circles\"
export TURTLEBOT_SIMULATION=\"false\"
export TURTLEBOT_3D_SENSOR=kinect
export TURTLEBOT_SERIAL_PORT=/dev/roomba
export GAZEBO_MODEL_DATABASE_URI=\"http://gazebosim.org/models\"
source $ROS_WORKSPACE/devel/setup.bash
"
echo "$BASH_CONFIG" >> $USERHOME/.bashrc
fi

sudo -H -u $USERNAME -i bash -c 'mkdir -p ~/ROS;'

popd

}

UBUNTU_DIST="trusty"
export ROS_DIST="indigo"

STATEFILE=/tmp/rosinstallstate

LAPTOP=1
STATE=`cat $STATEFILE`

if [ -f $STATEFILE ]; then
  export STATE=`cat $STATEFILE`
else
  export STATE=0
fi

set -e
if [[ $EUID -ne 0 ]]; then
  echo "You must be a root user" 2>&1
  exit 1
fi

SCRIPTDIR=`pwd`

RUNUSERNAME=$SUDO_USER

pushd /tmp

if (( $STATE < 1 )); then

  if [ ! -f /etc/apt/sources.list.d/ros-latest.list ]; then
    echo "deb http://packages.ros.org/ros/ubuntu $UBUNTU_DIST main" > /etc/apt/sources.list.d/ros-latest.list
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  fi

  PACKAGES="ros-$ROS_DIST-desktop-full ros-$ROS_DIST-rqt-* ros-$ROS_DIST-turtlebot ros-$ROS_DIST-turtlebot-apps ros-$ROS_DIST-turtlebot-create ros-$ROS_DIST-turtlebot-rviz-launchers ros-$ROS_DIST-audio-common ros-$ROS_DIST-usb-cam ros-$ROS_DIST-freenect-stack ros-$ROS_DIST-turtlebot-create-desktop ros-$ROS_DIST-turtlebot-dashboard ros-$ROS_DIST-turtlebot-gazebo ros-$ROS_DIST-gazebo-ros-control libfreenect-dev vim git subversion openssh-server chrony git-gui screen synaptic libzbar-dev python-rosinstall julius-voxforge libjulius-dev alsa-oss meld"

  if [ ! -f /etc/apt/sources.list.d/linrunner-tlp-$UBUNTU_DIST.list -a  -n $LAPTOP ]; then
    add-apt-repository -y ppa:linrunner/tlp
  fi

  if [ -n $LAPTOP ]; then
    PACKAGES="$PACKAGES tlp tlp-rdw smartmontools ethtool"
  fi

  apt-get -q update

  if [ -z "$PACKAGES" ]; then
    apt-get -y dist-upgrade
  else
    apt-get -y install $PACKAGES
  fi

  ntpdate ntp.ubuntu.com

  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
  fi
  rosdep update

fi
echo "1" > $STATEFILE

if (( $STATE < 2 )); then

  echo "enabled=0" > /etc/default/apport

  if [ -n $LAPTOP ]; then
    # Disable powersave measures for battery operation
    sh -c 'echo "TLP_ENABLE=1\nDISK_IDLE_SECS_ON_AC=0\nDISK_IDLE_SECS_ON_BAT=2\nMAX_LOST_WORK_SECS_ON_AC=15\nMAX_LOST_WORK_SECS_ON_BAT=60\nCPU_SCALING_GOVERNOR_ON_AC=ondemand\nCPU_SCALING_GOVERNOR_ON_BAT=ondemand\nCPU_BOOST_ON_AC=1\nCPU_BOOST_ON_BAT=0\nSCHED_POWERSAVE_ON_AC=0\nSCHED_POWERSAVE_ON_BAT=0\nNMI_WATCHDOG=0\nENERGY_PERF_POLICY_ON_AC=performance\nENERGY_PERF_POLICY_ON_BAT=normal\nSATA_LINKPWR_ON_AC=max_performance\nSATA_LINKPWR_ON_BAT=min_power\nPCIE_ASPM_ON_AC=performance\nPCIE_ASPM_ON_BAT=powersave\nWIFI_PWR_ON_AC=1\nWIFI_PWR_ON_BAT=1\nWOL_DISABLE=Y\nSOUND_POWER_SAVE_ON_AC=0\nSOUND_POWER_SAVE_ON_BAT=0\nSOUND_POWER_SAVE_CONTROLLER=N\nRUNTIME_PM_ON_AC=on\nRUNTIME_PM_ON_BAT=auto\nRUNTIME_PM_ALL=1\nRUNTIME_PM_DRIVER_BLACKLIST=\"radeon nouveau\"\nUSB_AUTOSUSPEND=1\nUSB_DRIVER_BLACKLIST=\"usbhid\"\nUSB_BLACKLIST_WWAN=1\nDEVICES_TO_DISABLE_ON_BAT_NOT_IN_USE=\"bluetooth\"\nUSB_BLACKLIST=\"045e:02ae 045e:02b0 045e:02ad\"" > /etc/default/tlp'
    service tlp restart
    # Disable sleep when lid is closed
    if ! grep -Fxq "HandleLidSwitch=ignore" /etc/systemd/logind.conf; then
       sh -c 'echo "HandleLidSwitch=ignore\n" > /etc/systemd/logind.conf'
       restart systemd-logind
    fi
  fi

  if [ -z `egrep -i '^turtlebot' /etc/group` ]; then
    addgroup turtlebot
  fi

  if ! [ -f /etc/udev/rules.d/52-turtlebot.rules ] ; then

    echo 'ATTRS{idProduct}=="2008",ATTRS{idVendor}=="0557",MODE="666",GROUP="turtlebot",SYMLINK="roomba"
ATTRS{idProduct}=="6001",ATTRS{idVendor}=="0403",MODE="666",GROUP="turtlebot",SYMLINK="roomba"' >> /etc/udev/rules.d/52-turtlebot.rules

    service udev restart
  fi
fi
echo "2" > $STATEFILE

if (( $STATE < 3 )); then

	if ! [ -f /usr/local/bin/roshost ] ; then
		wget -O /usr/local/bin/roshost https://raw.githubusercontent.com/vicoslab/vicos_ros/master/turtlebot_vicos/roshost.sh
	fi
  chmod +x /usr/local/bin/roshost

fi

echo "3" > $STATEFILE

configure_user $RUNUSERNAME > /dev/null

declare -a TEAMS=("alpha" "beta" "gamma" "delta" "epsilon" "zeta" "eta" "theta" "iota" "kappa" "lambda" "mu" "nu" "xi" "omicron" "pi" "rho")

SECRETLIST="$SCRIPTDIR/passwords.txt"

read -s -p "Enter password salt: " SALT
echo 

for TEAMNAME in "${TEAMS[@]}"
do
    echo "Creating user for team $TEAMNAME"
    configure_user "team_$TEAMNAME" > /dev/null
    echo "$USERNAME:$PASSWORD" >> $SECRETLIST
done

popd


