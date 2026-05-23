clear; clc; close all;

params = rig();
Simulink.Bus.createObject(params);
paramsBus = slBus1; clear slBus1;
sample_mission_plot