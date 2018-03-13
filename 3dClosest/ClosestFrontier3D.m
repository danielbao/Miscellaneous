function [movecount,k,nodecount] = ClosestFrontier3D(k,itr,number)
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
end
vox_sz = [1,1,1]; %length of cube sides
G.fig = figure(number);
set(gcf,'Renderer','OpenGL');
G.mapnum =27;%22;
G.movecount = 0;
G.movetyp = [-1,0,0;0,1,0;1,0,0;0,-1,0;0,0,1;0,0,-1;];
movecount=G.movecount;
G.drawflag=1; % Default 1, draw G.fig 'on'. Set 0 for draw G.fig 'off'.
G.videoflag=0;
G.playflag=1;
clc
%% Making a video demonstration. makemymovie gets the current frame of imge and adds to video file
format compact
MOVIE_NAME =['Leaf_Mapping_3D',num2str(G.mapnum),'_',num2str(k),'robots','_video']; %Change video name here
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

randRobots=randperm(numel(G.robvec)); %randRobots: 1:num_empty_spaces, randomly arranged
G.robvec(randRobots(k+1:end))=0; % locations of the k robots

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
        while(nnz(frontier_exp)>0)
            frontier_vec=G.boundvec; %current locations of frontiers
            roboloc=G.roboloc; %current locations of particles
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
    function rvec2 = applyMove(mv, rvecIn)
        rvec2 = zeros(size(rvecIn));
        % Move matrix is this
        % The key to using this is to have the right orientation set up
        % This is why axes are on now
        %1 -x left 
        %2 +x right 
        %3 +ydown?
        %4 -y up
        %5 +z
        %6 -z
        if mv==1 || mv==4 || mv==6 %collision check for left up and down (-z)
            for ni = 1:numel(rvecIn)
                if rvecIn(G.Moves(ni,mv)) ~= 1
                    rvec2(G.Moves(ni,mv)) = rvecIn(ni);
                    rvecIn(ni)=0;
                else
                    rvec2(ni)=rvecIn(ni);
                end
                rvecIn(ni)=rvec2(ni);
            end
        else %collision check for up and right
            for ni=numel(rvecIn):-1:1
                if rvecIn(G.Moves(ni,mv)) == 0
                    rvec2(G.Moves(ni,mv)) = rvecIn(ni);
                    rvecIn(ni)=0;
                else
                    rvec2(ni)=rvecIn(ni);
                end
            end
        end
    end

%% takes input from user
    function keyhandler(src,evnt) %#ok<INUSL>
        if strcmp(evnt.Key,'q')
            imwrite(flipud(get(G.axis,'CData')+1), G.colormap, '../../pictures/png/MatrixPermutePic.png');
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
        elseif strcmp(key,'n') || strcmp(key,'5') %+z
            mv = 5;
        elseif strcmp(key,'s') || strcmp(key,'6') %-z
            mv = 6;    
        %
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
                map_expected = circshift(map_expected,[0 0 1]);
            elseif mv==6
                map_expected = circshift(map_expected,[0 0 -1]);
            end
            %G.movecount = G.movecount+1;
            G.robvec = applyMove(mv, G.robvec);
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
    function t_Callback(~,~)%Joystick callback
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
    function movetomv(mv)%Used for joystick stuff
        if mv>0
            map_expected=G.im2;
            G.movecount = G.movecount+1;
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
            G.robvec = applyMove(mv, G.robvec);
            updateMap()
            updateTitle()
            drawnow
            if G.videoflag==1
                makemymovie()
            end
        end        
        
    end
%% Updating the map
    function updateMap()
        current_map = ones(size(G.obstacle_pos)); % 1== undiscovered
        current_map(G.free) = 2*G.robvec; %2 = particles on map
        
        G.im2=zeros(size(current_map)); %Set particles matrix to zero. This matrix will have 1 where particles are and zero otherwise
        G.im2(G.free(G.robvec>0))=1;
        RobotVisits=G.im2+RobotVisits; %RoboVisits stores cells already visited by particles
        robIndex = find(G.robvec);
        nodes(robIndex);
        current_map(RobotVisits==0)=1; % 1= undiscovered
        frontier_exp(current_map==2)=0;
        map_expected(current_map~=1)=0;
        mapped_obstacles = mapped_obstacles | map_expected;
        frontier_exp(mapped_obstacles)=0;
        current_map(frontier_exp==1)=4; % 4 = frontier cells
        current_map(mapped_obstacles==1)=3; % 3 = obstacles
        G.update_map=zeros(size(current_map));
        G.update_map(current_map==3)=1;
        G.update_map(current_map==1)=1;
        G.obstacles=find(mapped_obstacles);
        G.boundvec=find(current_map== 4);
        G.roboloc =find(current_map== 2);
        G.covered=find(current_map==0);
 clf       
 draw3d()
 
 [G.robscatx,G.robscaty,G.robscatz]=find(current_map== 2);
 %  scatter3(G.robscatx,G.robscaty,G.robscatz);
 %         colormap(G.colormap(unique(current_map)+1,:));
 
 
 if G.drawflag==0
     G.axis=imagesc(current_map); %show map that updates as robots explore
 end
    end
    function draw3d()
        figure(G.fig)
        [x,y,z]=ind2sub(size(G.update_map),G.obstacles);
        obspts=[x,y,z];
        % voxel_image(obspts,vox_sz,'g',0.1,'g');
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
        [x,y,z]=ind2sub(size(G.update_map),G.roboloc);
        RobotPts=[x,y,z];
        voxel_image(RobotPts, vox_sz,'r',1);
        
        axis equal
        axis on;
%         view([90 -90]);%2D Top-Down view
        view([97.6, -21.2]); %This view is great for the 3D viewing
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
                Moves(i,5) = world(r,c,h+1);
            end
            if blk(r,c,h-1) == 0
                Moves(i,6) = world(r,c,h-1);
            end
        end
        
    end
end