%% Analyzing Flight data from PX4 log
% by Will Wu
close all; clc; clear;

%% File name manipulation
log_name = "16_31_13_";
topics = ["actuator_controls_0_0" ;"actuator_outputs_0"; 
    "vehicle_angular_velocity_0"; "vehicle_attitude_0";
    "vehicle_attitude_setpoint_0"; "actuator_outputs_sim_0"];

for index = 1:length(topics)
    s.(topics(index)) = readtable(strcat(log_name, topics(index), ".csv"));
end

for index = 1:length(topics)
    figure;
    stackedplot(s.(topics(index)));
    title(topics(index)); 
end