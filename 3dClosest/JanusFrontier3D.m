function [movecount,k,nodecount,init_config] = JanusFrontier3D(k,itr,max_steps,map,config_flag,fill_flag,fill,p1,p2,p3,p4,p5,p6,starting_config)
% ClosestFrontier is a demonstration of mapping of a completely connected
% and bounded 2D discrete grid space with k particles that move uniformly.
% The permissible moves are left, right, up and down. Each move is one pixel
% in length. All particles move in the same direction unless stopped by
% black obstacles.
% The algorithm for motion planning moves the particles in red to frontier
% cells in blue until no frontier cells remain.
% Inputs: k- number of particles, itr= number of iterations
% outputs: movecount- number of moves taken for mapping, k= number of
% partilcles and nodecount- vector with number of frontier cells in each
% move.
% The DIJKSTRA distance is calculated in each cycle to find the shortest
% distance from all particles to the frontiers.
% There are 26 maps to choose from- 1 through 26. Map specifications can be
% found in BlockMaps

%  Aaron T. Becker
%     atbecker@uh.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global G;
if nargin<1
    k = 100;%num particles
    itr=1;
    number=1;
    map=0;
    max_steps=250;
    p1=0.25;
    p2=0.25;
    p3=0.25;
    p4=0.25;
    p5=0;
    p6=0;
    config_flag=0;
end
vox_sz = [1,1,1]; %length of cube sides
G.fig = figure(number);
set(gcf,'Renderer','OpenGL');
if map==0
    G.mapnum=27;%22
else
    G.mapnum=map;
end
G.movecount = 0;
G.movetyp = [-1,0,0;%-y
    0,1,0;%+x
    1,0,0;%+y
    0,-1,0;%-x
    0,0,1;%+z
    0,0,-1;];%-z
movecount=G.movecount;
G.drawflag=1; % Default 1, draw G.fig 'on'. Set 0 for draw G.fig 'off'.
G.videoflag=0;
G.playflag=1;
clc
%% Making a video demonstration. makemymovie gets the current frame of imge and adds to video file
format compact
MOVIE_NAME =['Leaf_Mapping_3D',num2str(G.mapnum),'_',num2str(k),'robots itr',num2str(itr),'_video with p1',num2str(p1)]; %Change video name here
writerObj = VideoWriter(MOVIE_NAME,'MPEG-4');%http://www.mathworks.com/help/matlab/ref/videowriterclass.html
set(writerObj,'Quality',100);
writerObj.FrameRate=30;
open(writerObj);
    function makemymovie()% Call after each frame is generated
        if G.videoflag==1
            figure(G.fig)
            F = getframe(G.fig);
            writeVideo(writerObj,F.cdata);
        end
    end
%% Setup map, matrices and initite mapping
[G.obstacle_pos,G.free,G.robvec,G.Moves] = SetupWorld(G.mapnum);
if(G.playflag==1)
%     % Define joystick ID (if only using 1 joystick, this will likely be '1')
%     ID = 1;
%     % Create joystick variable
%     joy=vrjoystick(ID);
%     t = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, 'TimerFcn', @t_Callback);
%     start(t);
%Uncomment this for joystick use!
end
set(G.fig ,'KeyPressFcn',@keyhandler,'Name','Massive Control','color','w')
G.maxX = size(G.obstacle_pos,2);
G.maxY = size(G.obstacle_pos,1);
G.colormap = [ 1,1,1; %Empty = white  0
    0.5,0.5,0.5; %undiscovered= grey 1
    1,0,0; %robot= white square with red circle 2
    0,0,0;%obstacle= black           3
    0,0,1;%boundary cells= blue      4
    ];

if config_flag==1%For using the same configurations as before
    G.robvec=starting_config;
end
randRobots=randperm(numel(G.robvec)); %randomize robots in their positions
%% Distribution of robots code
G.type1loc=zeros(size(G.robvec));
G.type2loc=zeros(size(G.robvec));
G.type3loc=zeros(size(G.robvec)); 
G.type4loc=zeros(size(G.robvec));
G.type5loc=zeros(size(G.robvec));
G.type6loc=zeros(size(G.robvec));

G.type1loc(randRobots(1:round(k*p1)))=1;
randRobots(1:round(k*p1))=[];
G.type2loc(randRobots(1:round(k*p2)))=1;
randRobots(1:round(k*p2)-1)=[];
G.type3loc(randRobots(1:round(k*p3)))=1;
randRobots(1:round(k*p3))=[];
G.type4loc(randRobots(1:round(k*p4)))=1;
randRobots(1:round(k*p4)-1)=[];
G.type1loc(randRobots(1:round(k*p5)))=1;
randRobots(1:round(k*p5))=[];
G.type1loc(randRobots(1:ceil(k*p6)))=1;
%Initialized by the probability distributions of each species
%6 species locations initialized!!
G.robvec=G.type1loc+G.type2loc+G.type3loc+G.type4loc+G.type5loc+G.type6loc;
init_config=G.robvec; % We store the first locations (linear indices) of the robots

map_1=zeros(size(G.obstacle_pos)); %Set zeros initially. We update the expected location of each particle in this map
map_2=zeros(size(G.obstacle_pos));
map_3=zeros(size(G.obstacle_pos));
map_4=zeros(size(G.obstacle_pos));
map_5=zeros(size(G.obstacle_pos));
map_6=zeros(size(G.obstacle_pos));

RobotVisits=zeros(size(G.obstacle_pos)); %Set blind map to zeros. We want to build path in this map
map_expected=zeros(size(G.obstacle_pos)); %Set zeros initially. We update the expected location if each particle in this map
mapped_obstacles=zeros(size(G.obstacle_pos)); %Map is updated when obstacles are found
frontier_exp= zeros(size(G.obstacle_pos)); %Map to update the locations of frontiers
updateMap() %Update the map with the information from all the seperate maps
set(gca,'box','off','xTick',[],'ytick',[],'ydir','normal','Visible','on');
axis equal
axis tight
updateTitle() %Update the values displayed in the title
hold on
makemymovie()



%End of initialization
if(G.playflag==0)
    CF()% Closest Frontier mapping algorithm
end
for rest=1:30
    view([-80-rest/3, 30+rest*2]);
    makemymovie()
    
end
 for rest=1:60
     view([-90, 90]);
     makemymovie()
     
 end
%% CF repeatedly moves particles to frontier cells until there are no more frontier cells left
    function CF()
        iter=1;
        while(nnz(frontier_exp)>0&&movecount<max_steps)
            frontier_vec=G.boundvec; %current locations of frontiers
            roboloc=G.roboloc; %Refresh local locations to global current locations of robots
            temp1loc=G.ind1;
            temp2loc=G.ind2;
            temp3loc=G.ind3;
            temp4loc=G.ind4;
            temp5loc=G.ind5;
            temp6loc=G.ind6;
            moveSeq = DijkstraForBoundary_mod3D(G.update_map,roboloc,frontier_vec); %the shortest path to a frontier cell selected by expanding from particles
            steps = min(inf,numel(moveSeq));
            for mvIn =1:steps
                moveto(moveSeq(mvIn)); %Implement moves on all particles
                updateTitle()
                nodecount(iter)=nnz(frontier_exp);
                iter=iter+1;
                makemymovie()
%                 if G.movecount==100
%                 pause()
%                 end
            end %end DFS
        end
        for i=1:5
            makemymovie()
        end
        
        
    end
%% nodes updates the frontier_exp which is the matrix of frontiers explored
    function nodes(robIndex)
        %For each robot, check if the cell in direction mv is unknown
        for mv_type=1:6
            for c = 1:numel(robIndex)
                i2 = G.ri(robIndex(c))+G.movetyp(mv_type,2);
                j2 = G.ci(robIndex(c))+G.movetyp(mv_type,1);
                h2= G.hi(robIndex(c))+G.movetyp(mv_type,3);
                % if the cell has neer been visited and isn't an obstacle
                if RobotVisits(i2,j2,h2) == 0 && mapped_obstacles(i2,j2,h2) == 0
                    frontier_exp(i2,j2,h2)=1;
                else
                    frontier_exp(i2,j2,h2)=0;
                end
            end
        end
    end


%% Apply move called by moveto to all the robots
    function [rvec1Out, rvec2Out, rvec3Out, rvec4Out,rvec5Out,rvec6Out] = applyMove(mv, rvec1In, rvec2In,rvec3In,rvec4In,rvec5In,rvec6In)
        rvec1Out = zeros(size(rvec1In));%Result matrix to store moved robots of type 1
        rvec2Out = zeros(size(rvec2In));
        rvec3Out = zeros(size(rvec3In));
        rvec4Out = zeros(size(rvec4In));
        rvec5Out = zeros(size(rvec5In));
        rvec6Out = zeros(size(rvec6In));
        switch mv
            case 1
                mv2=5;
                mv3=4;
                mv4=2;
                mv5=6;
                mv6=3;
            case 2
                mv2=6;
                mv3=3;
                mv4=1;
                mv5=5;
                mv6=4;
            case 3
                mv2=1;
                mv3=5;
                mv4=4;
                mv5=2;
                mv6=6;
            case 4
                mv2=2;
                mv3=6;
                mv4=3;
                mv5=1;
                mv6=5;
            case 5
                mv2=4;
                mv3=2;
                mv4=6;
                mv5=3;
                mv6=1;
            case 6
                mv2=3;
                mv3=1;
                mv4=5;
                mv5=4;
                mv6=2;
            otherwise
                disp('bad mv');
        end
        if mv==1 || mv==4 || mv==5%Collision check for left and down
            for ni = 1:numel(rvec1In)%G.Moves(ni,mv) returns 1 if there is a free space after the movement mv
                if rvec1In(G.Moves(ni,mv)) == 0 && rvec2In(G.Moves(ni,mv)) == 0 && rvec3In(G.Moves(ni,mv)) == 0 && rvec4In(G.Moves(ni,mv)) == 0 && rvec5In(G.Moves(ni,mv)) == 0 && rvec6In(G.Moves(ni,mv)) == 0 
                    %If there isn't any species at that location
                    %Move to that location
                    rvec1Out(G.Moves(ni,mv)) = rvec1In(ni);
                    rvec1In(ni)=0;%Clear previous location for robot
                else
                    rvec1Out(ni)=rvec1In(ni);%Keep old location
                end
                rvec1In(ni)=rvec1Out(ni);%Update old info with new location
            end
        else %Collision check for down and right
            for ni=numel(rvec1In):-1:1%For the rest of the robots
                if rvec1In(G.Moves(ni,mv)) == 0 && rvec2In(G.Moves(ni,mv)) == 0 && rvec3In(G.Moves(ni,mv)) == 0 && rvec4In(G.Moves(ni,mv)) == 0 && rvec5In(G.Moves(ni,mv)) == 0 && rvec6In(G.Moves(ni,mv)) == 0 
                    %Equivalent to ~=1
                    %If there isn't any robots at that location
                    rvec1Out(G.Moves(ni,mv)) = rvec1In(ni);%Set result to new location
                    rvec1In(ni)=0;%clear input for robot ni
                else
                    rvec1Out(ni)=rvec1In(ni);%Update result with old location
                end
                rvec1In(ni)=rvec1Out(ni);%Set old locations with new location
            end
        end
        %type 2 movement check
        if mv2==1 || mv2==4 || mv2==5 %Collision check for left and down
            for ni = 1:numel(rvec2In)%G.Moves(ni,mv) returns 1 if there is a free space after the movement mv
                if rvec1Out(G.Moves(ni,mv2)) == 0 && rvec2In(G.Moves(ni,mv2)) == 0 && rvec3In(G.Moves(ni,mv2)) == 0 && rvec4In(G.Moves(ni,mv2)) == 0 && rvec5In(G.Moves(ni,mv2)) == 0 && rvec6In(G.Moves(ni,mv2)) == 0
                    %If there isn't a species of type 1,2,3, or 4 that will move to
                    %that location
                    rvec2Out(G.Moves(ni,mv2)) = rvec2In(ni);%Set result to new location
                    rvec2In(ni)=0;%Clear previous location for robot
                else
                    rvec2Out(ni)=rvec2In(ni);%Keep old location
                end
                rvec2In(ni)=rvec2Out(ni);%Update old info with new location
            end
        else %Collision check for down and right
            for ni=numel(rvec2In):-1:1%For the rest of the robots descending
                if rvec1Out(G.Moves(ni,mv2)) == 0 && rvec2In(G.Moves(ni,mv2)) == 0 && rvec3In(G.Moves(ni,mv2)) == 0 && rvec4In(G.Moves(ni,mv2)) == 0 && rvec5In(G.Moves(ni,mv2)) == 0 && rvec6In(G.Moves(ni,mv2)) == 0
                    %Equivalent to ~=1
                    rvec2Out(G.Moves(ni,mv2)) = rvec2In(ni);%Set result to new location
                    rvec2In(ni)=0;%clear input for robot ni
                else
                    rvec2Out(ni)=rvec2In(ni);%Update result with old location
                end
                rvec2In(ni)=rvec2Out(ni);%Set old locations with new location
            end
        end
        %type 3 collision check
        if mv3==1 || mv3==4 || mv3==5%Collision check for left and down
            for ni = 1:numel(rvec3In)%G.Moves(ni,mv) returns 1 if there is a free space after the movement mv
                if rvec1Out(G.Moves(ni,mv3)) == 0 && rvec2Out(G.Moves(ni,mv3)) == 0 && rvec3In(G.Moves(ni,mv3)) == 0 && rvec4In(G.Moves(ni,mv3)) == 0 && rvec5In(G.Moves(ni,mv3)) == 0 && rvec6In(G.Moves(ni,mv3)) == 0
                    %If there isn't a species of type 1,2,3, or 4 that will move to
                    %that location
                    rvec3Out(G.Moves(ni,mv3)) = rvec3In(ni);%Set result to new location
                    rvec3In(ni)=0;%Clear previous location for robot
                else
                    rvec3Out(ni)=rvec3In(ni);%Keep old location
                end
                rvec3In(ni)=rvec3Out(ni);%Update old info with new location
            end
        else %Collision check for down and right
            for ni=numel(rvec3In):-1:1%For the rest of the robots descending
                if rvec1Out(G.Moves(ni,mv3)) == 0 && rvec2Out(G.Moves(ni,mv3)) == 0 && rvec3In(G.Moves(ni,mv3)) == 0 && rvec4In(G.Moves(ni,mv3)) == 0 && rvec5In(G.Moves(ni,mv3)) == 0 && rvec6In(G.Moves(ni,mv3)) == 0
                    %Equivalent to ~=1
                    rvec3Out(G.Moves(ni,mv3)) = rvec3In(ni);%Set result to new location
                    rvec3In(ni)=0;%clear input for robot ni
                else
                    rvec3Out(ni)=rvec3In(ni);%Update result with old location
                end
                rvec3In(ni)=rvec3Out(ni);%Set old locations with new location
            end
        end
        %type 4 collision check
        if mv4==1 || mv4==4 || mv4==5%Collision check for left and down
            for ni = 1:numel(rvec4In)%G.Moves(ni,mv) returns 1 if there is a free space after the movement mv
                if rvec1Out(G.Moves(ni,mv4)) == 0 && rvec2Out(G.Moves(ni,mv4)) == 0 && rvec3Out(G.Moves(ni,mv4)) == 0 && rvec4In(G.Moves(ni,mv4)) == 0 && rvec5In(G.Moves(ni,mv4)) == 0 && rvec6In(G.Moves(ni,mv4)) == 0
                    %If there isn't a species of type 1,2,3, or 4 that will move to
                    %that location
                    rvec4Out(G.Moves(ni,mv4)) = rvec4In(ni);%Set result to new location
                    rvec4In(ni)=0;%Clear previous location for robot
                else
                    rvec4Out(ni)=rvec4In(ni);%Keep old location
                end
                rvec4In(ni)=rvec4Out(ni);%Update old info with new location
            end
        else %Collision check for down and right
            for ni=numel(rvec4In):-1:1%Starting from the end
                if rvec1Out(G.Moves(ni,mv4)) == 0 && rvec2Out(G.Moves(ni,mv4)) == 0 && rvec3Out(G.Moves(ni,mv4)) == 0 && rvec4In(G.Moves(ni,mv4)) == 0 && rvec5In(G.Moves(ni,mv4)) == 0 && rvec6In(G.Moves(ni,mv4)) == 0
                    %Equivalent to ~=1
                    rvec4Out(G.Moves(ni,mv4)) = rvec4In(ni);%Set result to new location
                    rvec4In(ni)=0;%clear input for robot ni
                else
                    rvec4Out(ni)=rvec4In(ni);%Update result with old location
                end
                rvec4In(ni)=rvec4Out(ni);%Set old locations with new location
            end
        end
         %type 5 collision check
        if mv5==1 || mv5==4 || mv5==5%Collision check for left and down
            for ni = 1:numel(rvec5In)%G.Moves(ni,mv) returns 1 if there is a free space after the movement mv
                if rvec1Out(G.Moves(ni,mv5)) == 0 && rvec2Out(G.Moves(ni,mv5)) == 0 && rvec3Out(G.Moves(ni,mv5)) == 0 && rvec4In(G.Moves(ni,mv5)) == 0 && rvec5In(G.Moves(ni,mv5)) == 0 && rvec6In(G.Moves(ni,mv5)) == 0
                    %If there isn't a species of type 1,2,3, or 4 that will move to
                    %that location
                    rvec5Out(G.Moves(ni,mv5)) = rvec5In(ni);%Set result to new location
                    rvec5In(ni)=0;%Clear previous location for robot
                else
                    rvec5Out(ni)=rvec5In(ni);%Keep old location
                end
                rvec5In(ni)=rvec5Out(ni);%Update old info with new location
            end
        else %Collision check for down and right
            for ni=numel(rvec5In):-1:1%Starting from the end
                if rvec1Out(G.Moves(ni,mv5)) == 0 && rvec2Out(G.Moves(ni,mv5)) == 0 && rvec3Out(G.Moves(ni,mv5)) == 0 && rvec4In(G.Moves(ni,mv5)) == 0 && rvec5In(G.Moves(ni,mv5)) == 0 && rvec6In(G.Moves(ni,mv5)) == 0
                    %Equivalent to ~=1
                    rvec5Out(G.Moves(ni,mv5)) = rvec5In(ni);%Set result to new location
                    rvec5In(ni)=0;%clear input for robot ni
                else
                    rvec5Out(ni)=rvec5In(ni);%Update result with old location
                end
                rvec5In(ni)=rvec5Out(ni);%Set old locations with new location
            end
        end
          %type 6 collision check
        if mv6==1 || mv6==4 || mv6==5%Collision check for left and down
            for ni = 1:numel(rvec6In)%G.Moves(ni,mv) returns 1 if there is a free space after the movement mv
                if rvec1Out(G.Moves(ni,mv6)) == 0 && rvec2Out(G.Moves(ni,mv6)) == 0 && rvec3Out(G.Moves(ni,mv6)) == 0 && rvec4In(G.Moves(ni,mv6)) == 0 && rvec5In(G.Moves(ni,mv6)) == 0 && rvec6In(G.Moves(ni,mv6)) == 0
                    %If there isn't a species of type 1,2,3, or 4 that will move to
                    %that location
                    rvec6Out(G.Moves(ni,mv5)) = rvec6In(ni);%Set result to new location
                    rvec6In(ni)=0;%Clear previous location for robot
                else
                    rvec5Out(ni)=rvec6In(ni);%Keep old location
                end
                rvec6In(ni)=rvec5Out(ni);%Update old info with new location
            end
        else %Collision check for down and right
            for ni=numel(rvec6In):-1:1%Starting from the end
                if rvec1Out(G.Moves(ni,mv6)) == 0 && rvec2Out(G.Moves(ni,mv6)) == 0 && rvec3Out(G.Moves(ni,mv6)) == 0 && rvec4In(G.Moves(ni,mv6)) == 0 && rvec5In(G.Moves(ni,mv6)) == 0 && rvec6In(G.Moves(ni,mv6)) == 0
                    %Equivalent to ~=1
                    rvec5Out(G.Moves(ni,mv6)) = rvec6In(ni);%Set result to new location
                    rvec6In(ni)=0;%clear input for robot ni
                else
                    rvec6Out(ni)=rvec6In(ni);%Update result with old location
                end
                rvec6In(ni)=rvec6Out(ni);%Set old locations with new location
            end
        end
    end

%% takes input from user
    function keyhandler(src,evnt) %#ok<INUSL>
        if strcmp(evnt.Key,'q')
            imwrite(flipud(get(G.axis,'CData')+1), G.colormap, 'MatrixPermutePic.png');
        else
            moveto(evnt.Key)
        end
    end
%% moveto(key) calls apply move and updates the map
    function moveto(key)
        G.movecount = G.movecount+1;
        mv=0;
        if strcmp(key,'leftarrow') || strcmp(key,'l')|| strcmp(key,'1')  %-x
            mv = 1;
        elseif strcmp(key,'rightarrow')|| strcmp(key,'r')|| strcmp(key,'2')  %+x
            mv = 2;
        elseif strcmp(key,'uparrow')|| strcmp(key,'u')|| strcmp(key,'3')  %+y
            mv = 3;
        elseif strcmp(key,'downarrow')|| strcmp(key,'d') || strcmp(key,'4') %-y
            mv = 4;
        elseif strcmp(key,'n') || strcmp(key,'5') %-y
            mv = 5;
        elseif strcmp(key,'s') || strcmp(key,'6') %-y
            mv = 6;    
        end
        if mv>0
            map_expected=G.im2;
            
            % map_expected has 1 where robot is expected to be
            if mv==1
                map_expected = circshift(map_expected,[0 -1 0]);
            elseif mv==2
                map_expected = circshift(map_expected,[0 1 0]);
            elseif mv==3
                map_expected = circshift(map_expected,[1 0 0]);
            elseif mv==4
                map_expected = circshift(map_expected,[-1 0 0]);
            elseif mv==5
                map_expected = circshift(map_expected,[0 0 -1]);
            elseif mv==6
                map_expected = circshift(map_expected,[0 0 1]);
            end
            %G.movecount = G.movecount+1;
            [G.type1loc, G.type2loc, G.type3loc, G.type4loc, G.type5loc, G.type6loc]= applyMove(mv, G.type1loc, G.type2loc, G.type3loc, G.type4loc, G.type5loc, G.type6loc);%Move robots and put it in actual positions
            updateMap()
            updateTitle()
            if G.drawflag==1
%                  drawcirc()
            end
            drawnow
            if G.videoflag==1
               makemymovie() 
            end
        end
    end
    function t_Callback(~,~)
        try
            mv=0;%Reset move selector
            Y=axis(joy, 1);     % X-axis is joystick axis 1
            X=axis(joy, 2);     % Y-axis is joystick axis 2
            %Confusing but it currently works!
            up=button(joy, 5);  % Pressed R
            down=button(joy, 6);% Pressed L
            endflag=button(joy,9); %Pressed X
            reset=button(joy,10); %Reset button to redraw map
            if(reset>=0.1)
                [G.obstacle_pos,G.free,G.robvec,G.Moves] = SetupWorld(G.mapnum);
                randRobots=randperm(numel(G.robvec)); %randRobots: 1:num_empty_spaces, randomly arranged
                G.robvec(randRobots(k+1:end))=0; % locations of the k robots
                
                RobotVisits=zeros(size(G.obstacle_pos)); %Set blind map to zeros. We want to build path in this map
                map_expected=zeros(size(G.obstacle_pos)); %Set zeros initially. We update the expected location if each particle in this map
                mapped_obstacles=zeros(size(G.obstacle_pos)); %Map is updated when obstacles are found
                frontier_exp= zeros(size(G.obstacle_pos)); %Map to update the locations of frontiers
                updateMap() %Update the map with the information from all the seperate maps
            end
            if Y<=-0.1
                mv = 2;
            elseif Y>=0.1
                mv = 1;
            elseif X<=-0.1
                mv = 3;
            elseif X>=0.1
                mv = 4;
            elseif up>=0.1
                mv = 5;
            elseif down>=0.1
                mv = 6;
            end
            if(endflag>=0.1)
                stop(t);
            end
            movetomv(mv);
        catch
            joy = vrjoystick(1);
            disp('Error');
            return;
        end
    end
    function movetomv(mv)
        if mv>0%Do the move as seen from the 
            %global reference frame based on the type given
            type1temp=G.type1loc;
            type2temp=G.type2loc;
            type3temp=G.type3loc;
            type4temp=G.type4loc;
            type5temp=G.type5loc;
            type6temp=G.type6loc;
            %These store the locations and performs the respective shifts
            
            map_1=G.im2==1;
            map_2=G.im2==2;
            map_3=G.im2==3;
            map_4=G.im2==4;
            map_5=G.im2==5;
            map_6=G.im2==6;
            %This gets a logical matrix which we can perform circ shift on
            map_expected=G.im2;%im2 is the local particle location matrix
            %updates every call to updateMap()
            G.movecount = G.movecount+1;%Increment movecount everytime a move is applied
            %Approach to solve this problem
            %Merge the arrays (add them)
            %If they overlap (value>1) then only one of them moves to that
            %new location
            %The one that moves is determined by the type that moves right
            %and up first because those are the ones we check first?
            %i.e we have a global command of left
            %We must move types 3 and 4 first in order to get the right
            %ordering of the particles
            %If there's overlap we can trace backwards to see which
            %particle overlapped into that space and then keep that
            %particle from moving!
            if mv==1
                map_1 = circshift(map_1,[0 -1 0]);%shift map left by 1
                map_2 = circshift(map_2,[0 0 -1]);%shift type 2 map down by 1
                map_3 = circshift(map_2,[-1 0 0]);%shift type 2 map down by 1
                map_4 = circshift(map_3,[0 1 0]);
                map_5 = circshift(map_5,[0 0 1]);
                map_6 = circshift(map_6,[1 0 0]);
            elseif mv==2
                map_1 = circshift(map_1,[0 1 0]);%shift map right by 1
                map_2 = circshift(map_2,[0 0 1]);
                map_3 = circshift(map_3,[1 0 0]);
                map_4 = circshift(map_4,[0 -1 0]);
                map_5 = circshift(map_5,[0 0 -1]);
                map_6 = circshift(map_6,[-1 0 0]);
            elseif mv==3
                map_1 = circshift(map_1,[1 0 0]);%shift map up by 1
                map_2 = circshift(map_2,[0 -1 0]);
                map_3 = circshift(map_3,[0 0 -1]);
                map_4 = circshift(map_4,[-1 0 0]);
                map_5 = circshift(map_5,[0 1 0]);
                map_6 = circshift(map_6,[0 0 1]);
                
            elseif mv==4
                map_1 = circshift(map_1,[-1 0 0]);%shift map down by 1
                map_2 = circshift(map_2,[0 1 0]);
                map_3 = circshift(map_3,[0 0 1]);
                map_4 = circshift(map_4,[1 0 0]);
                map_5 = circshift(map_5,[0 -1 0]);
                map_6 = circshift(map_6,[0 0 -1]);
            elseif mv==5
                map_1 = circshift(map_1,[0 0 -1]);%shift map down by 1
                map_2 = circshift(map_2,[-1 0 0]);
                map_3 = circshift(map_3,[0 1 0]);
                map_4 = circshift(map_4,[0 0 1]);
                map_5 = circshift(map_5,[1 0 0]);
                map_6 = circshift(map_6,[0 -1 0]);
            elseif mv==6
                map_1 = circshift(map_1,[0 0 1]);%shift map down by 1
                map_2 = circshift(map_2,[1 0 0]);
                map_3 = circshift(map_3,[0 -1 0]);
                map_4 = circshift(map_4,[0 0 -1]);
                map_5 = circshift(map_5,[-1 0 0]);
                map_6 = circshift(map_6,[0 1 0 ]);
            end
            
            [G.type1loc, G.type2loc, G.type3loc, G.type4loc, G.type5loc, G.type6loc]= applyMove(mv, G.type1loc, G.type2loc, G.type3loc, G.type4loc, G.type5loc, G.type6loc);%Move robots and put it in actual positions
            updateMap()
            updateTitle()
            if G.videoflag==1
                makemymovie()
            end
        end        
        
    end
%% Updating the map
    function updateMap()
        current_map = ones(size(G.obstacle_pos)); % 1== undiscovered
        current_map(G.free) = 11*G.type1loc + 12*G.type2loc + 13*G.type3loc + 14*G.type4loc +15*G.type5loc +16*G.type6loc; %11=type 1 particles on map
        realmap=flipud(current_map);
        %Each type is marked by 1+ their type number
        G.im2=zeros(size(current_map)); %Set particles matrix to zero. This matrix will have 1 where particles are and zero otherwise
        G.im2(G.free(G.type1loc>0))=1;%Set particles matrix to 1 where the robots are
        G.im2(G.free(G.type2loc>0))=2;
        G.im2(G.free(G.type3loc>0))=3;
        G.im2(G.free(G.type4loc>0))=4;
        G.im2(G.free(G.type5loc>0))=5;
        G.im2(G.free(G.type6loc>0))=6;
        G.visits(G.free(G.type1loc>0))=1;%Set other particles matrix to 1 where the robots are
        G.visits(G.free(G.type2loc>0))=1;
        G.visits(G.free(G.type3loc>0))=1;
        G.visits(G.free(G.type4loc>0))=1;
        G.visits(G.free(G.type5loc>0))=1;
        G.visits(G.free(G.type6loc>0))=1;
        robIndex = G.type1loc+G.type2loc+G.type3loc+G.type4loc+G.type5loc+G.type6loc;%Get positions of robots again
        robIndex = find(robIndex);
        RobotVisits=G.im2+RobotVisits; %RoboVisits stores cells already visited by particles
        nodes(robIndex);
        current_map(RobotVisits==0)=1; % 1= undiscovered
        frontier_exp(current_map==11 | current_map ==12 | current_map ==13 | current_map ==14 | current_map ==15 | current_map ==16)=0;
        map_expected(current_map~=1)=0;
        map_1(current_map~=1)=0;
        map_2(current_map~=1)=0;
        map_3(current_map~=1)=0;
        map_4(current_map~=1)=0;
        map_5(current_map~=1)=0;
        map_6(current_map~=1)=0;
        mapped_obstacles = mapped_obstacles | map_expected | map_1 | map_2 | map_3 | map_4| map_5 | map_6;%OR the expected and mapped obstacles to update obstacles
        frontier_exp(mapped_obstacles)=0;
        current_map(frontier_exp==1)=4; % 4 = frontier cells
        current_map(mapped_obstacles==1)=3; % 3 = obstacles
        G.ind1=find(current_map==11);
        G.ind2=find(current_map==12);
        G.ind3=find(current_map==13);
        G.ind4=find(current_map==14);
        G.ind5=find(current_map==15);
        G.ind6=find(current_map==16);
        G.update_map=zeros(size(current_map));
        G.update_map(current_map==3)=1;
        G.update_map(current_map==1)=1;
        G.obstacles=find(mapped_obstacles);
        G.boundvec=find(current_map== 4);
        G.roboloc =find(current_map== 2);
        G.covered=find(current_map==0);
 clf       
 draw3d()
 
%  [G.robscatx,G.robscaty,G.robscatz]=find(current_map== 2);
 %  scatter3(G.robscatx,G.robscaty,G.robscatz);
 %         colormap(G.colormap(unique(current_map)+1,:));
 
 
%  if G.drawflag==0
%      G.axis=imagesc(current_map); %show map that updates as robots explore
%  end
    end
    function draw3d()
        figure(G.fig)
%         [x,y,z]=ind2sub(size(G.update_map),G.obstacles);
%         obspts=[x,y,z];
%         voxel_image(obspts,vox_sz,'g',0.1,'g');
        hold on
        locs=find(G.update_map==1);
        [x,y,z]=ind2sub(size(G.update_map),locs);
        GoalPts=[x,y,z];
        voxel_image(GoalPts, vox_sz,[0.9,0.9,0.9],0);
        [x,y,z]=ind2sub(size(G.update_map),G.boundvec);
        frontier=[x,y,z];
        voxel_image(frontier, vox_sz,'b',0.3);
        [x,y,z]=ind2sub(size(G.update_map), G.covered);
        covered=[x,y,z];
        voxel_image(covered, vox_sz,[1 1 1],0.1,'r');
        [x,y,z]=ind2sub(size(G.update_map),G.ind1);
        Robot1Pts=[x,y,z];
        voxel_image(Robot1Pts, vox_sz,'r',1);
        [x,y,z]=ind2sub(size(G.update_map),G.ind2);
        Robot2Pts=[x,y,z];
        voxel_image(Robot2Pts, vox_sz,'g',1);  
        [x,y,z]=ind2sub(size(G.update_map),G.ind3);
        Robot3Pts=[x,y,z];
        voxel_image(Robot3Pts, vox_sz,[0.5 0 1],1);
        [x,y,z]=ind2sub(size(G.update_map),G.ind4);
        Robot4Pts=[x,y,z];
        voxel_image(Robot4Pts, vox_sz,'c',1);
        [x,y,z]=ind2sub(size(G.update_map),G.ind5);
        Robot5Pts=[x,y,z];
        voxel_image(Robot5Pts, vox_sz,'m',1);   
        [x,y,z]=ind2sub(size(G.update_map),G.ind6);
        Robot6Pts=[x,y,z];
        voxel_image(Robot6Pts, vox_sz,'y',1);
        axis equal
        axis off;
        view([-80, 30]);
        hold off
        drawnow()
    end
%% Update title when called
    function updateTitle()
        if nnz( frontier_exp)==1
            FC=' frontier Cell';
        else
            FC=' frontier Cells';
        end
        title([num2str(G.movecount), ' moves, ',num2str(sum(G.robvec)),' particles, ', num2str(nnz( frontier_exp)), FC, ', ', num2str(nnz(G.free)), ' free cells'])
    end
%% setup map
    function [blk,free,robvec,Moves] = SetupWorld(mapnum)
        %  blk is the position of obstacles
        % free is the index of the free spaces in blk
        % robvec is a binary vector where the ith element is true if there
        % is a robot at free(i).
        blk = blockMaps(mapnum); % function returns the binary map specified by mapnum
        free = find(blk==0);
        robvec = ones(size(free));
        [ri,ci,hi] = ind2sub(size(blk),free);
        G.ri=ri;
        G.ci=ci;
        G.hi=hi;
        Moves = repmat( (1:numel(free))',1,6);
        world = -blk;
        world(free) = 1:numel(free);
        for i = 1:numel(free)
            r = ri(i);
            c = ci(i);
            h=  hi(i);
            if blk(r,c-1,h) == 0
                Moves(i,1) = world(r,c-1,h);
            end
            if blk(r,c+1,h) == 0
                Moves(i,2) = world(r,c+1,h);
            end
            if blk(r+1,c,h) == 0
                Moves(i,3) = world(r+1,c,h);
            end
            if blk(r-1,c,h) == 0
                Moves(i,4) = world(r-1,c,h);
            end
            if blk(r,c,h+1) == 0
                Moves(i,6) = world(r,c,h+1);
            end
            if blk(r,c,h-1) == 0
                Moves(i,5) = world(r,c,h-1);
            end
        end
        
    end
end