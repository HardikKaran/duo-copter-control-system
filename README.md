# duo-copter-control-system
Derive fundamental physics of the cart system using either Lagrangian method or Newton's laws. Build physics engine in MATLAB. Inputs: actual height (measured by LiDAR sensors) and reference height. Output: required throttle setting (0-100) for the control response. Cascaded PID feedback loop. Kalman filter. Monte Carlo simulations for robustness.

When opening for the first time:
1. run in the command window: params = rig()
2. generate a simulink bus for the controller: Simulink.Bus.createObject(params);
3. change name of bus to: paramsBus = slBus1; clear slBus1;
4. run sample_mission_plot.m
5. open the simulink file controller.slx