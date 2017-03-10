close all; clear all;
% x0 = [0, 0, 0, 0.2, 0, 0, 0, 0];
x0 = [0, 0, 0, 0, 0, 0, 0, 0];
resolution = 20;
waypoints = build_stabilized_trajectory(x0, resolution)


