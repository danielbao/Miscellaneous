% This code seeks to map a workspace using the MATLAB Robotics
% Toolbox that is provided in r2017A. This uses modified default code
% that is adapted for a swarm of robots. 
% 
% 
%
%
% Prerequisites:
% Robotics Toolbox 
% Run rosinit
% Author: Daniel Bao
% E-mail: dzbao@uh.edu

%Code to test the default simulator with one robot on the map.
%Simulator=ExampleHelperRobotSimulator('simpleMap',2);
Simulator=RobotSimulator('simpleMap',2,2);
enableROSInterface(Simulator, true);
%exampleHelperTurtleBotKeyBoardControl();
% Simulator.LaserSensor.NumReadings=50;
% scanSub = rossubscriber('scan');
% [velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');
% tftree = rostf;
% pause(1);