# CarND-P10-ModelPredictiveControl

CarND-P10-ModelPredictiveControl implements Model Predictive Control
that computes vehicle actuator values that optimizes its
trajectory based on the vehicle kinetic model.

## File Structure
### C++ Source Files - /src
- **[main.cpp](src/main.cpp)** - runs WebSocket server to interact with 
    the Term 2 simulator.  It receives vehicle waypoints (the desired path)
    and its current state (position, angle, and speed).  It returns the
    computed steering and throttle values back to the simulator.
- **[MPC.cpp](src/MPC.cpp)** - implements Model Predictive Control.
    For a given vehicle state and the polynomial fit for the waypoints,
    it returns actuator values and predicted trajectories.
- **[Eigen/](src/Eigen/)** - C++ library for linear algebra, matrix and
    vector opeartions
### Project Report
- **[writeup_report.md](writeup_report.md)** - a report discussing the
    model implementations.
### Other Support Files
- **[CMakeLists.txt](CMakeLists.txt)** - CMake file
- **[DATA.md](DATA.md)** - describes the incoming data format from the
    simulator. 

## Getting Started
### [Download ZIP](https://github.com/gabeoh/CarND-P10-ModelPredictiveControl/archive/master.zip) or Git Clone
```
git clone https://github.com/gabeoh/CarND-P10-ModelPredictiveControl.git
```

### Install uWebSockets
Run the installation script from the project repository. 
- [uWebSockets](https://github.com/uNetworking/uWebSockets)
#### Linux Installation
```
./install-ubuntu.sh
```
#### Mac Installation
```
./install-mac.sh
```

### Install Ipopt and CppAD
Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.

### Download Simulator
- [Udacity Self-Driving Car - Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases/)

### Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

### Build and Run
1. Make a build directory: `mkdir build && cd build`
1. Generate Makefile: `cmake ..`
1. Compile: `make` 
1. Run it: `./mpc`

## License
Licensed under [MIT](LICENSE) License.
