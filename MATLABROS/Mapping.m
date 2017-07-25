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
Simulator=RobotSimulator('simpleMap',2,20);
enableROSInterface(Simulator, true);
% for i=1:Simulator.N
%     Simulator.LaserSensor(i).NumReadings=50;
% end
% scanSub = rossubscriber('scan');
% [velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');
tftree = rostf;
% pause(1);



%% Makeshift driving controller; random mapping with n set of moves
% This code makes a binaryOccupancyGrid with an empty map where the laser
% readings can be inserted and form a structured map.
prompt = 'How many random moves do you want me to make?';
moves = input(prompt);
map = robotics.OccupancyGrid(14,13,20);
figureHandle = figure('Name', 'Map');
axesHandle = axes('Parent', figureHandle);
mapHandle = show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Update 0');
updateCounter = 1;
controlRate=robotics.Rate(50);
while(moves > 0)
    %We are going to get the laser readings from the Robots
    %Then we insert them into the map.
    %Then we select a random move from the available headings
    for i=1:Simulator.N
        [ranges, angles]=Simulator.getRangeData(i);
        ranges(isnan(ranges))=Simulator.LaserSensor(i).MaxRange;
        insertRay(map, Simulator.getRobotPose(i), ranges, angles, Simulator.LaserSensor(i).MaxRange);
    end
    r=randi([1 4], 1,1);
    omega=0;
    switch r
        case 1
            omega=0;
        case 2
            omega=pi/2;
        case 3
            omega=pi;
        case 4
            omega=3*pi/2; 
    end
    if ~mod(updateCounter,2)
        mapHandle.CData = occupancyMatrix(map);
        title(axesHandle, ['OccupancyGrid: Update ' num2str(updateCounter)]);
    end
    Simulator.drive(1,omega);
    updateCounter = updateCounter+1;
    moves=moves-1;
    waitfor(controlRate);
    pause(1);
end

%% Stuff for movie-making
% format compact
% MOVIE_NAME =['MATLABROS_Mapping_video']; %Change video name here
% writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
% set(writerObj,'Quality',100);
% writerObj.FrameRate=30;
% open(writerObj);
%     function makemymovie()% Call after each frame is generated
%         if videoflag==1
%             figure(Simulator.Figure)
%             F = getframe(Simulator.Figure);
%             writeVideo(writerObj,F.cdata);
%         end
%     end