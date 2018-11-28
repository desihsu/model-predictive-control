# Model Predictive Control

Uses [model predictive control](https://en.wikipedia.org/wiki/Model_predictive_control) with a kinematic vehicle model to drive a car around a track.

## Dependencies

* [Ipopt and CppAD](./install_Ipopt_CppAD.md)

## Build Instructions

1. Clone this repo
2. Download [simulator](https://github.com/udacity/self-driving-car-sim/releases/)
3. Install [uWebSocketIO](https://github.com/uNetworking/uWebSockets): ```~$ sh install-ubuntu.sh```
4. ```~$ mkdir build && cd build```
5. ```~$ cmake .. && make``` 

## Usage

1. Run the simulator (MPC Controller)
2. Connect: ```~$ ./mpc```

## Kinematic Vehicle Model

<a href="https://www.codecogs.com/eqnedit.php?latex=\\x_{t&plus;1}&space;=&space;x_t&space;&plus;&space;v_t&space;\cdot&space;\cos(\psi_t)&space;\cdot&space;dt\\&space;y_{t&plus;1}&space;=&space;y_t&space;&plus;&space;v_t&space;\cdot&space;\sin(\psi_t)&space;\cdot&space;dt\\&space;\psi_{t&plus;1}&space;=&space;\psi_t&space;&plus;&space;v_t&space;\cdot&space;\delta_t&space;\cdot&space;dt&space;/&space;L_f&space;\\&space;v_{t&plus;1}&space;=&space;v_t&space;&plus;&space;a_t&space;\cdot&space;dt\\&space;cte_{t&plus;1}&space;=&space;f(x_t)&space;-&space;y_t&space;&plus;&space;v_t&space;\cdot&space;\sin(e\psi_t)&space;\cdot&space;dt\\&space;e\psi_{t&plus;1}&space;=&space;\psi_t&space;-&space;\psi&space;des_t&space;&plus;&space;v_t&space;\cdot&space;\delta_t&space;\cdot&space;dt&space;/&space;L_f" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\\x_{t&plus;1}&space;=&space;x_t&space;&plus;&space;v_t&space;\cdot&space;\cos(\psi_t)&space;\cdot&space;dt\\&space;y_{t&plus;1}&space;=&space;y_t&space;&plus;&space;v_t&space;\cdot&space;\sin(\psi_t)&space;\cdot&space;dt\\&space;\psi_{t&plus;1}&space;=&space;\psi_t&space;&plus;&space;v_t&space;\cdot&space;\delta_t&space;\cdot&space;dt&space;/&space;L_f&space;\\&space;v_{t&plus;1}&space;=&space;v_t&space;&plus;&space;a_t&space;\cdot&space;dt\\&space;cte_{t&plus;1}&space;=&space;f(x_t)&space;-&space;y_t&space;&plus;&space;v_t&space;\cdot&space;\sin(e\psi_t)&space;\cdot&space;dt\\&space;e\psi_{t&plus;1}&space;=&space;\psi_t&space;-&space;\psi&space;des_t&space;&plus;&space;v_t&space;\cdot&space;\delta_t&space;\cdot&space;dt&space;/&space;L_f" title="\\x_{t+1} = x_t + v_t \cdot \cos(\psi_t) \cdot dt\\ y_{t+1} = y_t + v_t \cdot \sin(\psi_t) \cdot dt\\ \psi_{t+1} = \psi_t + v_t \cdot \delta_t \cdot dt / L_f \\ v_{t+1} = v_t + a_t \cdot dt\\ cte_{t+1} = f(x_t) - y_t + v_t \cdot \sin(e\psi_t) \cdot dt\\ e\psi_{t+1} = \psi_t - \psi des_t + v_t \cdot \delta_t \cdot dt / L_f" /></a>