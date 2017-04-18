# Yuneec SITL Simulation

## Prerequisites

### Gazebo7

#### Mac

Tested on macOS 10.11 and 10.12.

Make sure to have [homebrew](http://brew.sh).

```
brew tap px4/simulation
brew update
brew install gazebo7
```

#### Ubuntu

Tested on Ubuntu 16.04.

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo7 libgazebo7-dev
```

Instructions taken from [gazebosim.org](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install).

## Run it

Use the included bash script to start the simulation environment:

```
cd Yuneec-SITL-simulation-xxx/
./typhoon_sitl.bash
```

To run it without simulation GUI, use:

```
HEADLESS=1 ./typhoon_sitl.bash
```
