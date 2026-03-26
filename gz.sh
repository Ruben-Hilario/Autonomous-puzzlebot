sudo apt-get update
sudo apt-get install lsb-release curl gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
sudo apt-get update
sudo apt-get install gz-garden
sudo apt-get install ros-humble-ros-gzgarden
