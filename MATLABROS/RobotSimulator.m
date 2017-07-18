classdef RobotSimulator < handle
    %   Modified version of ExampleHelperRobotSimulator for research
    %   purposes
    %   OBJ = RobotSimulator creates a simple simulator for a
    %   differential-drive robot. If ROS interface is enabled, the robot
    %   receives velocity commands (messages of type 'geometry_msgs/Twist')
    %   on the '/mobile_base/commands/velocity' topic, and sends odometry
    %   information (messages of type 'nav_msgs/Odometry') to the '/odom'
    %   topic. In addition, it updates a figure window showing the current
    %   position of the robot.
    %
    %   OBJ = RobotSimulator(MAPNAME) loads a predefined map
    %   in the simulator. MAPNAME is a string and can be one of these:
    %   'simpleMap' (default), 'emptyMap', 'borderMap', or 'complexMap'.
    %   The maps are assumed to have a resolution of 2 cells / meter.
    %
    %   OBJ = RobotSimulator(MAPNAME,MAPRESOLUTION) loads a
    %   predefined map with MAPNAME and sets the resolution to
    %   MAPRESOLUTION (in cells / meter). By default, the resolution is 2
    %   cells / meter.
    %
    %   OBJ = RobotSimulator(BOG) loads the map defined by the
    %   robotics.BinaryOccupancyGrid object BOG into the simulator.
    %
    %   Notes:
    %   1) RobotSimulator's ROS interface expects MATLAB ROS
    %   functionality to be initialized. You can do this by calling ROSINIT
    %   (for a local ROS master) or invoking ROSINIT(MASTER_IP) to connect
    %   to an external master.
    %
    %   2) To stop the simulator, close the associated figure window or
    %      delete the object.
    %
    %   When ROS interface is enabled, you can control the motion of the
    %   simulated robot via ROS. The following topics and services are
    %   handled by the simulator:
    %
    %   RobotSimulator Published Topics:
    %       /odom - Current robot odometry information (pose and velocity)
    %       /scan - The data from the simulated laser scanner
    %       /mobile_base/sensors/bumper - Indicates if the robot hits an obstacle
    %       /ground_truth_pose - Ground truth robot pose
    %
    %   RobotSimulator Subscribed Topics:
    %       /mobile_base/commands/velocity - Velocity commands to the robot
    %
    %   RobotSimulator Services:
    %       /sim/new_robot_pose - Move the robot into a new starting pose
    %       /sim/reset_poses    - Reset the pose of the robot to the last known starting pose
    %       /gazebo/reset_world - Reset the pose of the robot to the last known starting pose
    %
    %   Some other behavior of the simulator can be changed by accessing
    %   various properties.
    %
    %   RobotSimulator properties:
    %
    %       Map             - Map to be used by the robot
    %       Robot           - Object representing a differential drive robot.
    %       LaserSensor     - Object representing the simulated range sensor.
    %       HasROSInterface - Indicates whether object has ros interface.
    %       HasFigureWindow - Indicates whether object has GUI interface.
    %       HasLaser        - Indicates whether simulated robot has Laser sensor.
    %       PlotTrajectory  - Indicates whether robot's trajectory is plotted.
    %
    %   RobotSimulator methods:
    %
    %       enableLaser        - Setup the laser sensor interface.
    %       enableROSInterface - Setup ROS publisher, subscriber and service server.
    %       showTrajectory     - Plot the trajectory of the robot.
    %       setRobotSize       - Set the robot's bounding radius equal to size.
    %       setRobotPose       - Set robot at specific pose in world frame.
    %       getRobotPose       - Returns ground truth robot pose in world frame.
    %       getRobotOdom       - Returns robot's odometry data.
    %       drive              - Drives robot with linear velocity v and angular velocity omega.
    %       getRangeData       - Returns laser scan data in ranges and angles.
    %       delete             - Close simulator.
    %       randomizeLocation  - Set the robot position to a random location.
    %       resetSimulation    - Set the robot position to initial conditions.
    %
    %
    %     % Enable laser sensor
    %     enableLaser(robotsim,true);
    %
    %     % Plot robot trajectory
    %     showTrajectory(robotsim,true);
    %

    %   See also robotics.BinaryOccupancyGrid, ExampleHelperSimRangeSensor, ExampleHelperSimDifferentialDriveRobot.
    
    properties(Constant, Access = protected)
        %Step - Integration step size (in seconds)
        %   This is also the approximate rate of odometry and ground truth
        %   pose message publishing, which also depends on MATLAB callback
        %   queue.
        Step = 0.05
        
        %PlotInterval - Interval between plot updates (in seconds)
        %   The exact plot interval is also affected by MATLAB callback
        %   queue.
        PlotInterval = 0.1
        
        %FigureName - Name of simulation figure window
        FigureName = 'Robot Simulator'
        
        %FigureTag - Tag used for the simulation figure window.
        %   This is used to check if the window is already open.
        FigureTag = 'RobotSimulator'
    end
    
    properties 
        %Robot - Object representing differential drive robots
        Robot
        %LaserSensor - Object representing simulated range sensors
        LaserSensor
        %HasROSInterface - Indicates whether object has ros interface
        HasROSInterface = false
        %HasFigureWindow - Indicates whether object has GUI interface
        HasFigureWindow
        %HasLaser - Indicates whether simulated robot has Laser sensor
        HasLaser
        %PlotTrajectory - Indicates whether robot's trajectory is plotted.
        PlotTrajectory = false
        %NUM_Robots - number of robots planned to simulate!
        N
    end
    properties (Hidden)
        %Axes - Handle to main plot axes
        Axes
    end
    properties (Dependent)
        %Map - Map to be used by the robot
        Map
    end
    properties (Access = {?RobotSimulator})
        %InternalMap - The map stored inside simulator
        InternalMap
        %KinematicsTimer - Timer for kinematics integration
        KinematicsTimer = timer.empty
        %PlotTimer - Timer triggering plot updates
        PlotTimer = timer.empty
        % Simulated Robot
        %InitialRobotState - Initial state of robot
        %   This is a 3-vector with elements [x y theta]
        InitialRobotState = [0 0 0]
        %RobotBodyFaceColor - Robot's body color
        RobotBodyFaceColor = [0.9 0.0 0.0]
        %RobotBodyTriangleVertices - Robot's body in an arrow shape
        RobotBodyTriangleVertices = [[-0.3, -0.05,-0.3,0.8]; [-0.5,0,0.5,0]] * 0.5;
        %RobotSize - Radius of bounding circle for robot's body
        RobotSize = 0.4;
        %TrajectoryBufferCapacity - The capacity of the circular buffer
        TrajectoryBufferCapacity = 5000;
        %TrajectoryBuffer - Stores trajectory of robot in a circular buffer
        TrajectoryBuffer        
        %TrajectoryIndex - Pointer to the last element in the trajectory buffer
        TrajectoryIndex = 0;        
        %TrajectoryUpdateTolerance - Trajectory buffer only gets updated if
        %robot's pose change exceeds TrajectoryUpdateTolerance
        TrajectoryUpdateTolerance = 0.01;        
        % Laser scanner simulation        
        %ScanRanges - The latest simulated range readings
        ScanRanges        
        %ScanAngles - The latest scan angles (corresponding to ScanRanges)
        ScanAngles        
        %ScanCollisionLoc - The latest 2D locations for laser hits
        %   These are the locations where the laser beams intersect
        %   obstacles in the map.
        ScanCollisionLoc
        % ROS
        % ROS node
        %GlobalNode - Handler to Global ROS node
        GlobalNode
        % ROS publishers; specific to each robot with the /robot_n topic
        % name
        %LaserScanPublisher - Publisher for /scan topic
        LaserScanPublisher      
        %BumperStatePublisher - Publisher for /mobile_base/sensors/bumper topic
        BumperStatePublisher      
        %VelCmdPublisher - Publisher for /mobile_base/commands/velocity messages
        VelCmdPublisher        
        %OdometryPublisher - Publisher for /odom topic
        OdometryPublisher        
        %GroundTruthPosePublisher - Publisher for /ground_truth_pose topic
        GroundTruthPosePublisher        
        %TransformTree - Transformation tree
        TransformTree        
        % ROS messages        
        %LaserScanMessage - Laser scan reading published on /scan
        %   Message type is sensor_msgs/LaserScan
        LaserScanMessage        
        %BumperStateMessage - std_msgs/Bool message indicating bumper / collision state
        BumperStateMessage       
        %LastVelocityCmd - Last valid velocity command that was received
        LastVelocityCmd        
        %PoseMessage - nav_msgs/Odometry message for /odom and /ground_truth_pose topic
        PoseMessage       
        %TransformMessage - Transformation message
        TransformMessage        
        % ROS subscribers        
        %VelCmdSubscriber - Subscriber for /mobile_base/commands/velocity messages
        VelCmdSubscriber        
        % ROS services        
        %ResetSimulationService - Service server for /sim/reset_poses
        ResetSimulationService        
        %RandomizeLocationService - Service server for /sim/new_robot_pose
        RandomizeLocationService        
        %GazeboResetModelPosesService - Service server for /gazebo/reset_world
        GazeboResetModelPosesService
        % Graphics 
        %MyFigureTag - Unique figure tag belonging to the current simulator
        %session
        MyFigureTag
        %Figure - Handle to figure window
        Figure = []
        %RobotBodyHandle - Handle to robot body graphical representation
        RobotBodyHandle = []     
        %ScanLineHandles - Graphics handles for laser beam visualization
        ScanLineHandles
        %ScanPointHandles - Graphics handles for laser end point visualization
        ScanPointHandles
        %TrajectoryHandle - Graphics handle for robot's trajectory
        TrajectoryHandle = []
        %RandomLocationButton - Button for randomizing robot location
        RandomLocationButton
        %ResetSimulationButton - Button for resetting the robot simulation
        ResetSimulationButton
    end
    methods
        function obj = RobotSimulator(varargin)
            %RobotSimulator Constructor for simulator object
            %   Please check the class definition for details.
            obj.MyFigureTag = RobotSimulator.FigureTag;
            if ~isempty(findobj('type', 'figure', 'tag', obj.MyFigureTag))
                error('RobotSimulator:RobotSimulatorAlreadyExists',...
                    'A robot simulator is already running, please close it by deleting the simulator from workspace and closing the GUI window.');
            end
            % Load all example maps from MAT file, 
            % change this part if you
            % want to change where the maps are loaded from
            examplesFilePath = 'SimulatorMaps.mat';
            exampleMaps = load(examplesFilePath);
            % Parse arguments to constructor
            narginchk(0,3);
            switch nargin
                case 0
                    % RobotSimulator() syntax
                    % Use default values
                    mapName = 'emptyMap';
                    mapResolution = 2;
                    obj.InternalMap = robotics.BinaryOccupancyGrid(exampleMaps.(mapName), mapResolution);
                    obj.InternalMap.GridLocationInWorld = [-12.5,-12.5];
                case 1
                    % RobotSimulator(MAPNAME) or
                    % RobotSimulator(BOG) syntax
                    map = varargin{1};
                    if ischar(map)
                        % RobotSimulator(MAPNAME) syntax
                        mapResolution = 2;
                        mapName = validatestring(map, fieldnames(exampleMaps), 'RobotSimulator', 'mapName');
                        obj.InternalMap = robotics.BinaryOccupancyGrid(exampleMaps.(mapName), mapResolution);
                    else
                        % RobotSimulator(BOG) syntax
                        validateattributes(map, {'robotics.BinaryOccupancyGrid'}, {'nonempty','scalar'}, ...
                            'RobotSimulator', 'bog');
                        obj.InternalMap = map;
                    end
                case 2
                    % RobotSimulator(MAPNAME, MAPRESOLUTION) syntax
                    mapName = varargin{1};
                    mapResolution = varargin{2};
                    mapName = validatestring(mapName, fieldnames(exampleMaps), 'RobotSimulator', 'mapName');
                    validateattributes(mapResolution, {'double'}, {'nonempty','scalar'}, 'RobotSimulator', 'mapResolution');
                    obj.InternalMap = robotics.BinaryOccupancyGrid(exampleMaps.(mapName), mapResolution);
                case 3
                    % RobotSimulator(MAPNAME, MAPRESOLUTION, NUM_ROBOTS)
                    % syntax
                    mapName = varargin{1};
                    mapResolution = varargin{2};
                    mapName = validatestring(mapName, fieldnames(exampleMaps), 'RobotSimulator', 'mapName');
                    validateattributes(mapResolution, {'double'}, {'nonempty','scalar'}, 'RobotSimulator', 'mapResolution');
                    obj.InternalMap = robotics.BinaryOccupancyGrid(exampleMaps.(mapName), mapResolution);
                    obj.N=varargin{3};
                otherwise
                    error('RobotSimulator:InvalidInput',...
                        'Invalid number of arguments to RobotSimulator.');
            end
            
            % Initialize trajectory buffer
            obj.TrajectoryBuffer = NaN(obj.TrajectoryBufferCapacity,3);
%             RobotBodyHandle(obj.N)=struct;
%             obj.RobotBodyHandle = RobotBodyHandle;
            Robot(obj.N)=DifferentialDriveRobot();
            obj.Robot=Robot;
            PoseMessage(obj.N) = rosmessage('nav_msgs/Odometry');
            obj.PoseMessage=PoseMessage;
            LaserScanMessage(obj.N) = rosmessage('sensor_msgs/LaserScan');
            obj.LaserScanMessage=LaserScanMessage;
            BumperStateMessage(obj.N) = rosmessage('std_msgs/Bool');
            obj.BumperStateMessage=BumperStateMessage;
            TransformMessage(obj.N) = rosmessage('geometry_msgs/TransformStamped');
            obj.TransformMessage=TransformMessage;
            for i=1:obj.N
                % Set initial state for robot
                obj.Robot(i) = DifferentialDriveRobot();
                obj.Robot(i).setPose(obj.InitialRobotState);
                obj.PoseMessage(i) = rosmessage('nav_msgs/Odometry');
                obj.LaserScanMessage(i) = rosmessage('sensor_msgs/LaserScan');
                obj.BumperStateMessage(i) = rosmessage('std_msgs/Bool');
                obj.TransformMessage(i) = rosmessage('geometry_msgs/TransformStamped');
                obj.TransformMessage(i).Header.FrameId = 'map/robot_';
                obj.TransformMessage(i).ChildFrameId = 'robot_base';
            end          
            setupFigure(obj);
            obj.HasFigureWindow = true;
            
            setupLaserScanner(obj);
            obj.HasLaser = true;
%            randomizeLocation(obj);
            % Start the two timing loops
            obj.KinematicsTimer = ExampleHelperROSTimer(obj.Step, ...
                @obj.updateKinematics);
            obj.PlotTimer = ExampleHelperROSTimer(obj.PlotInterval, ...
                @obj.updatePlot);
            
            % Randomize location of robot and reset simulation
            randomizeLocation(obj);
            resetSimulation(obj);
        end
        
        function enableLaser(obj, hasLaser)
            %enableLaser Setup the laser sensor interface
            %   enableLaser(OBJ, true) - Enable the robot's laser sensor.
            %   enableLaser(OBJ, false) - Disable the robot's laser sensor.
            
            validateattributes(hasLaser,{'numeric', 'logical'}, ...
                {'scalar','nonempty','binary'}, 'enableLaser', 'enableLaser');
            
            if hasLaser == obj.HasLaser
                return
            end
            
            if hasLaser
                setupLaserScanner(obj);
                obj.HasLaser = true;
            else
                obj.HasLaser = false;
            end
        end
        
        function showTrajectory(obj, showTrajectory)
            %showTrajectory Plot the trajectory of the robot
            %   showTrajectory(OBJ, true) - Plot the robot's trajectory.
            %   showTrajectory(OBJ, false) - Do not plot the robot's trajectory.
            validateattributes(showTrajectory, {'numeric','logical'},{'scalar','nonempty','binary'},'showTrajectory','showTrajectory');
            obj.PlotTrajectory = showTrajectory;
        end
        
        function setRobotSize(obj, radius)
            %setRobotSize Set the robot's bounding radius equal to size
            %   setRobotSize(OBJ, r) - Bound the robot's body volume in a
            %   circle with radius r.
            validateattributes(radius, {'numeric'}, {'nonempty', 'scalar', 'positive', 'nonnan', 'finite', 'real'}, 'setRobotSize', 'radius');
            radius = double(radius);
            obj.RobotBodyTriangleVertices = obj.RobotBodyTriangleVertices*radius/obj.RobotSize;
            obj.RobotSize = radius;
            obj.plotRobot()
        end
        
        function delete(obj)
            %delete Called on object destruction or when the simulator window is closed
            %   delete(OBJ) - Stop timers first (otherwise callbacks may be
            %   invoked with invalid Subscriber and Publisher) and then
            %   close the GUI.
            
            if ~isempty(obj.KinematicsTimer) && isvalid(obj.KinematicsTimer)
                obj.KinematicsTimer.Timer.stop;
                delete(obj.KinematicsTimer);
            end
            
            if ~isempty(obj.PlotTimer) && isvalid(obj.PlotTimer)
                obj.PlotTimer.Timer.stop;
                delete(obj.PlotTimer);
            end
            
            if obj.HasROSInterface
                obj.enableROSInterface(false);
            end
            
            if ~isempty(obj.Figure)
                delete(obj.Figure);
            end
            
        end
        
        function setRobotPose(obj, pose, n)
            %setRobotPose Set robot at specific pose in world frame.
            %   setRobotPose(OBJ, [x, y ,theta]) - Set the robot's pose to
            %   [x, y, theta] in world frame.
            validateattributes(pose, {'numeric'}, {'nonempty', 'vector', 'numel', 3, 'nonnan', 'finite', 'real'}, 'setRobotPose', 'pose');
            pose = double(pose);
            obj.Robot(n).setPose(pose);
            obj.InitialRobotState = reshape(pose,1,3);
            [obj.ScanRanges(:,n), obj.ScanAngles(:,n), obj.ScanCollisionLoc(:,:,n)] = obj.LaserSensor(n).getReading(pose);
            obj.TrajectoryBuffer = NaN(obj.TrajectoryBufferCapacity, 3);
            obj.TrajectoryIndex = 0;
        end
        
        function pose = getRobotPose(obj, n)
            %getRobotPose Returns ground truth robot pose in world frame.
            %   POSE = getRobotPose(OBJ) - Returns world frame ground truth
            %   pose in [x, y, theta] vector form.
            if isempty(obj.Robot(n))
                pose = [];
                return;
            end
            
            pose = obj.Robot(n).Pose;
        end
        
        function odom = getRobotOdom(obj, n)
            %getRobotOdom Returns robot's odometry data.
            %   ODOM = getRobotOdom(OBJ) - Returns ODOM in [x,y,theta]
            %   vector form Odometry data is measured relative to robot's
            %   initial body frame.
            robotpose = getRobotPose(obj, n);
            yaw = obj.InitialRobotState(3);
            rotm = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1];
            odom = (rotm*(robotpose-obj.InitialRobotState)')';
        end
        
        function drive(obj, v, omega, n)
            %drive Drives robot velocity commands
            %   drive(OBJ, v, omega, n) - Drives the nth robot with linear
            %   velocity v and angular velocity omega.
            validateattributes(v, {'double'}, {'nonempty', 'scalar', 'nonnan', 'finite', 'real'}, 'drive', 'v');
            validateattributes(omega, {'double'}, {'nonempty', 'scalar', 'nonnan', 'finite', 'real'}, 'drive', 'omega');
            
            % Set velocity commands for robot. They will be executed in the
            % next simulation loop.
            obj.Robot(n).setVelocityCommand([v, omega]);
            
        end
        
        function [ranges, angles] = getRangeData(obj, n)
            %getRangeData Returns laser scan data in ranges and angles.
            %   [RANGES, ANGLES] = getRangeData(obj) - Returns laser scan
            %   measurements in terms of two vectors RANGES and ANGLES,
            %   each element in RANGES corresponds to the distance
            %   measurement in the direction specified in ANGLES relative
            %   to robot's body frame. The robot's front direction is 0
            %   angle and robot's right hand side is negative direction.
            ranges = [];
            angles = [];
            
            % Get robot pose
            robotPose = getRobotPose(obj, n);
            if isempty(robotPose) || isempty(obj.LaserSensor)
                return;
            end
            
            % Retrieve laser readings at current robot pose
            ranges = obj.LaserSensor(n).getReading(robotPose);
            angles = obj.LaserSensor(n).AngleSweep.';
        end
        
        function randomizeLocation(obj)
            %randomizeLocation Set the robots' pose to a random location
            %   randomizeLocation(OBJ) - Set the robots' position to a random
            %   location. This function ensures that the robot position is
            %   in free space.
            for i=1:obj.N
                isOccupied = true;
                iterations = 100;
                x=0;y=0;theta=0;
                while isOccupied && iterations > 0
                    x = obj.InternalMap.XWorldLimits(1) + rand(1)*diff(obj.InternalMap.XWorldLimits);
                    y = obj.InternalMap.YWorldLimits(1) + rand(1)*diff(obj.InternalMap.YWorldLimits);
                    theta = rand(1)*pi - pi/2;
                    isOccupied = obj.InternalMap.getOccupancy([x y]);
                    iterations = iterations - 1;                
                end

                % Update the initial robot state
                obj.setRobotPose([x,y,theta], i);                
            end
            
        end
        
        function resetSimulation(obj)
            %resetSimulation Set the robot pose to initial conditions
            %   resetSimulation(OBJ) - Set robot's pose to the initial
            %   conditions. Set all velocity commands to zero as well.
            
            if obj.HasROSInterface && isvalid(obj.GlobalNode)
                % Make sure that robot velocity is set to zero
                velmsg = rosmessage(obj.VelCmdPublisher);
                send(obj.VelCmdPublisher, velmsg);
                % Reset the initial robot state
                for i=1:obj.N                   
                    obj.setRobotPose(obj.InitialRobotState, i);
                    obj.TrajectoryBuffer = NaN(obj.TrajectoryBufferCapacity, 3);
                    obj.TrajectoryIndex = 0;
                end
            end      
        end
        
        function enableROSInterface(obj, enableROS)
            %enableROSInterface Setup ROS publisher, subscriber and service
            %server.
            %   enableROSInterface(OBJ, true) - Enable ROS interface.
            %   enableROSInterface(OBJ, false) - Disable ROS interface.
            validateattributes(enableROS,{'numeric', 'logical'}, ...
                {'scalar','nonempty','binary'}, 'enableROSInterface', 'enableROS');
            
            if enableROS == obj.HasROSInterface
                return;
            end
            
            if enableROS
                try
                    obj.GlobalNode = robotics.ros.internal.Global.getNodeHandle(false);
                catch
                    obj.GlobalNode = [];
                    error('RobotSimulator:EnableROSbeforeROSINIT',...
                        'ROS interface cannot be enabled before ROS initialization in MATLAB, please call ''rosinit'' first.');
                end
            end
%             TransformTree(obj.N)=rostf;
            obj.TransformTree=rostf; %#ok<*PROPLC>
            if enableROS && ~isempty(obj.GlobalNode)
                % Create ROS subscribers and publishers                
                for i=1:obj.N
                    VelCmdSubscriber(i) = rossubscriber(strcat('/mobile_base/commands/velocity/robot_',num2str(i)), 'geometry_msgs/Twist', ...
                        @obj.newTwistMessage);
                    VelCmdPublisher(i) = rospublisher(strcat('/mobile_base/commands/velocity/robot_',num2str(i)), 'geometry_msgs/Twist');
                    OdometryPublisher(i) = rospublisher(strcat('/odom/robot_',num2str(i)), 'nav_msgs/Odometry');
                    GroundTruthPosePublisher(i) = rospublisher(strcat('/ground_truth_pose/robot_',num2str(i)), 'nav_msgs/Odometry');
                    LaserScanPublisher(i) = rospublisher(strcat('/scan/robot_',num2str(i)), 'sensor_msgs/LaserScan');
                    BumperStatePublisher(i) = rospublisher(strcat('/mobile_base/sensors/bumper/robot_',num2str(i)), 'std_msgs/Bool');                   
                end
                obj.VelCmdSubscriber=VelCmdSubscriber;
                obj.OdometryPublisher=OdometryPublisher;
                obj.GroundTruthPosePublisher=GroundTruthPosePublisher;
                obj.LaserScanPublisher=LaserScanPublisher;
                obj.BumperStatePublisher=BumperStatePublisher;
                
                try
                    % Create services for resetting the simulation
                    % Both /sim/reset_poses and /gazebo/reset_world services will
                    % do the same thing (reset the robot's start position and
                    % velocity). The /gazebo/reset_world service is made available
                    % to preserve parity with the simulated TurtleBot
                    % representation in Gazebo.
                    obj.ResetSimulationService = rossvcserver( '/sim/reset_poses', 'std_srvs/Empty', ...
                        @obj.resetSimulationServiceCallback);
                    obj.GazeboResetModelPosesService = rossvcserver( '/gazebo/reset_world', 'std_srvs/Empty', ...
                        @obj.resetSimulationServiceCallback);
                    
                    % Create service for randomizing robot location
                    obj.RandomizeLocationService = rossvcserver('/sim/new_robot_pose', 'std_srvs/Empty', ...
                        @obj.randomizeLocationServiceCallback);
                    
                    obj.HasROSInterface = true;
                catch
                    error('RobotSimulator:ServicesAlreadyExist',...
                        'Services with same name is already created. Possibly due to existing robot simulator.');
                end
            else
                obj.HasROSInterface = false;            
                % Don't create any publishers/subscribers if there is no
                % ROS interface
            end
        end
        
        function enableFigureWindow(obj, showFig)
            %enableFigureWindow Setup simulator GUI.
            %   enableFigureWindow(OBJ, true) - Open simulator GUI.
            %   enableFigureWindow(OBJ, false) - Close simulator GUI and
            %   stop simulator.
            validateattributes(showFig,{'numeric', 'logical'}, ...
                {'scalar','nonempty','binary'}, 'showFigureWindow', 'showFig');
            
            if showFig == obj.HasFigureWindow
                return;
            end
            
            savedWindowSetting = obj.HasFigureWindow;
            
            try
                if showFig
                    obj.setupFigure;
                    obj.PlotTimer.Timer.start;
                    
                    obj.HasFigureWindow = true;
                else
                    obj.HasFigureWindow = false;
                    delete(obj);
                end
            catch ex
                obj.HasFigureWindow = savedWindowSetting;
                rethrow(ex);
            end
        end
        
    end
    
    %% Custom setters and getters for class properties
    methods
        function set.Map(obj, newMap)
            %set.Map Setter for Map property
            validateattributes(newMap,{'robotics.BinaryOccupancyGrid'}, ...
                {'scalar','nonempty'}, '', 'Map');
            obj.InternalMap = copy(newMap);
            obj.drive(0,0);
            obj.randomizeLocation();
            
            hold(obj.Axes, 'off');
            obj.Map.show('world', 'Parent', obj.Axes);
            hold(obj.Axes, 'on');
            title(obj.Axes, '');
            
            RobotBodyHandle(obj.N)=fill(0,0,[1 1 1]); %#ok<*CPROPLC>
            obj.RobotBodyHandle = RobotBodyHandle;
            obj.TrajectoryHandle = [];
            obj.ScanLineHandles = gobjects(obj.N);
            obj.ScanPointHandles = gobjects(obj.N);
            obj.plotRobot();
            obj.LaserSensor.Map = obj.InternalMap;
        end
        
        function map = get.Map(obj)
            %get.Map Getter for Map property
            map = copy(obj.InternalMap);
        end
    end
    
    %%
    methods (Access = private)
        
        function newTwistMessage(obj, ~, msg)
            %newTwistMessage A callback function that listens to
            % '/mobile_base/commands/velocity' and send velocity command
            % to robot whenever a new message is received.
            if isvalid(obj)
                for i=1:obj.N
                    obj.Robot(i).setVelocityCommand(msg);
                end
            end
        end
        
        function setupLaserScanner(obj)
            %setupLaserScanner Setup the simulated laser scanner
            LaserSensor(obj.N)=SimRangeSensor();
            obj.LaserSensor=LaserSensor; 
            N_Laser=obj.LaserSensor(obj.N).NumReadings;
            ScanAngles=zeros(N_Laser,obj.N,1);
            obj.ScanAngles=ScanAngles;
            ScanRanges=zeros(N_Laser,obj.N,1);
            obj.ScanRanges=ScanRanges;
            ScanCollisionLoc=zeros(N_Laser,obj.N,2);
            obj.ScanCollisionLoc=ScanCollisionLoc;
            obj.ScanLineHandles=gobjects(obj.N);
            obj.ScanPointHandles=gobjects(obj.N);
             %#ok<*PROP>
            for i=1:obj.N
                obj.LaserSensor(i) = SimRangeSensor();
                obj.LaserSensor(i).Map = obj.InternalMap;
                senseAngles = obj.LaserSensor(i).AngleSweep;  
                % Set values for laser scan message that are the same for each
                % simulation step
                msg = obj.LaserScanMessage(i);
                msg.Header.FrameId = strcat('laser_scanner/robot_',num2str(i));
                msg.AngleMin = senseAngles(1);
                msg.AngleMax = senseAngles(end);
                msg.TimeIncrement = 0;
                msg.ScanTime = obj.Step;
                msg.RangeMin = 0;
                msg.RangeMax = obj.LaserSensor(i).MaxRange;
            end
        end
        
        function setupFigure(obj)
            %setupFigure Setup figure window
            
            obj.Figure = figure('DeleteFcn', @(~,~)obj.enableFigureWindow(false), 'Name', obj.FigureName, 'tag', obj.MyFigureTag);
            ax = axes('parent', obj.Figure);
            
            obj.InternalMap.show('world', 'Parent', ax);
            hold(ax, 'on');
            title(ax, '');
            obj.Axes = ax;
            
            % Create buttons in the GUI window
            obj.RandomLocationButton = uicontrol(obj.Figure, ...
                'Style', 'pushbutton', ...
                'String','Randomize Location', ...
                'Units', 'normalized', ...
                'Position', [0.80 0.01 0.2 0.05], ...
                'Callback', @obj.randomLocationButtonCallback);
            
            obj.ResetSimulationButton = uicontrol(obj.Figure, ...
                'Style', 'pushbutton', ...
                'String','Reset Simulation', ...
                'Units', 'normalized', ...
                'Position', [0.0 0.01 0.2 0.05], ...
                'Callback', @obj.resetSimulationButtonCallback);
            
            obj.plotRobot();
            
        end
        
        function updateKinematics(obj, ~, ~)            
            for i=1:obj.N
                %updateKinematicsUpdate The kinematic model of the robot
                % Save last pose
                initialPose = obj.Robot(i).Pose;
                obj.Robot(i).updateKinematics(obj.Step);

                % Update robot state
                newPose = obj.Robot(i).Pose;

                % Check for collision with map
                try
                    isOccupied = obj.InternalMap.getOccupancy([newPose(1) newPose(2)]);
                catch
                    % Cannot go outside the world
                    isOccupied = true;
                end

                if isOccupied
                    % Hit a obstacle. If we treat it as a "bounce" (i.e., move
                    % the robot backwards), that causes noisy bumper state, so
                    % don't update at all and set it back to its initial state.
                    obj.BumperStateMessage(i).Data = true;
                    obj.Robot(i).setPose(initialPose);
                else
                    obj.BumperStateMessage(i).Data = false;
                end

                if obj.HasLaser
                    % Update range readings
                    pose = obj.Robot(i).Pose;                    
                    [obj.ScanRanges(:,i), obj.ScanAngles(:,i), obj.ScanCollisionLoc(:,:,i)] = obj.LaserSensor(i).getReading(pose);
                end

                % Check whether MATLAB ROS is still running.
                % If the global node no longer exist, stop publishing ROS
                % message
                if obj.HasROSInterface
                    obj.HasROSInterface = isvalid(obj.GlobalNode);
                end

                if obj.HasROSInterface
                    %Changed for multi-robot systems
                    send(obj.BumperStatePublisher(i), obj.BumperStateMessage(i));
                    sendPoseMessage(obj,obj.OdometryPublisher(i), obj.getRobotOdom(i), i);
                    sendPoseMessage(obj,obj.GroundTruthPosePublisher(i), obj.getRobotPose(i), i);

                    if obj.HasLaser
                        % Update range reading
                        obj.LaserScanMessage(i).Header.Stamp = rostime('now');
                        obj.LaserScanMessage(i).AngleIncrement = abs(angdiff(obj.ScanAngles(2,i), obj.ScanAngles(1,i)));
                        % Maybe we need to change the scanAngles to have
                        % values for all robots
                        obj.LaserScanMessage(i).RangeMax = obj.LaserSensor(i).MaxRange;
                        obj.LaserScanMessage(i).Ranges = obj.ScanRanges(:,i);
                        obj.LaserScanMessage(i).Header.Seq = obj.LaserScanMessage(i).Header.Seq + 1;
                        obj.LaserScanMessage(i).Header.Stamp = rostime('now');
                        send(obj.LaserScanPublisher(i), obj.LaserScanMessage(i));
                    end

                    % Send transform to tf
                    robotPose = obj.getRobotPose(i);
                    obj.TransformMessage(i).Transform.Translation.X = robotPose(1);
                    obj.TransformMessage(i).Transform.Translation.Y = robotPose(2);
                    quat = eul2quat([robotPose(3) 0 0], 'ZYX');
                    obj.TransformMessage(i).Transform.Rotation.W = quat(1);
                    obj.TransformMessage(i).Transform.Rotation.X = quat(2);
                    obj.TransformMessage(i).Transform.Rotation.Y = quat(3);
                    obj.TransformMessage(i).Transform.Rotation.Z = quat(4);
                    obj.TransformMessage(i).Header.Stamp = rostime('now');
                    obj.TransformTree.sendTransform(obj.TransformMessage(i));
                end

                obj.updateTrajectoryBuffer(obj.getRobotPose(i));
            end
            
        end
        
        function updateTrajectoryBuffer(obj, pose)
            %updateTrajectoryBuffer Insert pose into trajectory buffer.
            %Currently not updated for multi-robot systems; May need more
            %modifications
            validateattributes(pose,{'numeric'},{'nonempty','row','numel',3},'updateTrajectoryBuffer','pose');
            pose = double(pose);
            if obj.TrajectoryIndex == 0
                obj.TrajectoryIndex = obj.TrajectoryIndex + 1;
                obj.TrajectoryBuffer(obj.TrajectoryIndex,:) = pose;
            end
            
            if norm(pose-obj.TrajectoryBuffer(obj.TrajectoryIndex,:)) > obj.TrajectoryUpdateTolerance
                obj.TrajectoryIndex = mod(obj.TrajectoryIndex, obj.TrajectoryBufferCapacity) + 1;
                obj.TrajectoryBuffer(obj.TrajectoryIndex,:) = pose;
            end
        end
        
        
        function updatePlot(obj, ~, ~)
            %updatePlot Update the figure window with current robot
            %   Also plot the simulated range sensor rays.
            % Seems OK for the multi-robot stuff
            if ~isgraphics(obj.Axes)
                return
            end
            
            obj.plotRobot();
            
            if obj.PlotTrajectory
                obj.plotTrajectory();
            end
            
            % Plot laser scan readings
            if obj.HasLaser
                obj.plotLaser();
            end
            drawnow('limitrate');
        end
        
        function plotRobot(obj)
            %plotRobot Plot robot
            %Needs modification to support redrawing of the robot objects
            %once they are randomized and occasional robots that do get
            %stuck on the origin
            obj.RobotBodyHandle = gobjects(obj.N);
            for i=1:obj.N
                x = obj.Robot(i).Pose(1);
                y = obj.Robot(i).Pose(2);
                theta = obj.Robot(i).Pose(3);

%                 if isempty(obj.RobotBodyHandle(i)) || ~isgraphics(obj.RobotBodyHandle(i))
                    currentMarkerColor = obj.RobotBodyFaceColor;
%                 else
%                     currentMarkerColor = obj.RobotBodyHandle(i).FaceColor;
%                 end

                if ~isempty(obj.BumperStateMessage(i)) && obj.BumperStateMessage(i).Data
                    % Bumping into an obstacle. Alternate the color of the
                    % marker between red and white
                    if all(currentMarkerColor == obj.RobotBodyFaceColor)
                        markerFaceColor = [1 0 0];
                    elseif all(currentMarkerColor == [1 0 0])
                        markerFaceColor = obj.RobotBodyFaceColor;
                    end
                else
                    markerFaceColor = obj.RobotBodyFaceColor;
                end
                R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
                verticesPos = R * obj.RobotBodyTriangleVertices;
                %Must make sure that the RobotBody Handle isn't a graphics
                %object yet?
                if isempty(obj.RobotBodyHandle(i)) || ~isgraphics(obj.RobotBodyHandle(i))
                    obj.RobotBodyHandle(i) = fill(verticesPos(1,:).' + x, verticesPos(2,:).' + y, 'w','LineWidth', 1);
                    obj.RobotBodyHandle(i).FaceColor = markerFaceColor;
                else
                    obj.RobotBodyHandle(i).Vertices = [verticesPos(1,:).' + x, verticesPos(2,:).' + y];
                    obj.RobotBodyHandle(i).FaceColor = markerFaceColor;
                end
                uistack(obj.RobotBodyHandle(i), 'top');
            end            
        end
        
        function plotLaser(obj)
            for j=1:obj.N
                %plotLaser Plot laser readings
                %Needs a way to allocate array when there is a CollisionLoc
                %object required
                x = obj.Robot(j).Pose(1);
                y = obj.Robot(j).Pose(2);

                % Update range reading
                angles = obj.ScanAngles(:,j);
                collisionLoc = obj.ScanCollisionLoc(:,:,j);

                % If there are no existing plot handles, create them
                if any(~isgraphics(obj.ScanLineHandles(j))) || isempty(obj.ScanLineHandles(j))
                    obj.ScanLineHandles(j) = plot(obj.Axes, [x 0], [y 0], 'b');
                end

                xplot = zeros(1, 3*length(angles));
                yplot = zeros(1, 3*length(angles));
                for i = 1:length(angles)
                    idx1 = 3*i-2;
                    idx2 = idx1+1;
                    idx3 = idx2+1;

                    xplot(idx1:idx3) = [x collisionLoc(1) NaN];
                    yplot(idx1:idx3) = [y collisionLoc(2) NaN];
                end
                obj.ScanLineHandles(j).XData = xplot;
                obj.ScanLineHandles(j).YData = yplot;
            end            
        end
        
        function plotTrajectory(obj)
            %plotTrajectory Plot robot trajectory
            %Needs update for multi-robot stuff
            if obj.TrajectoryIndex == 0
                return
            end
            
            trajectory = [obj.TrajectoryBuffer(obj.TrajectoryIndex:end,:);obj.TrajectoryBuffer(1:obj.TrajectoryIndex-1,:)];
            
            if isempty(obj.TrajectoryHandle) || ~isgraphics(obj.TrajectoryHandle)
                obj.TrajectoryHandle = plot(obj.Axes, trajectory(:,1), trajectory(:,2),'LineWidth',3);
            else
                obj.TrajectoryHandle.XData = trajectory(:,1);
                obj.TrajectoryHandle.YData = trajectory(:,2);
            end
            
            uistack(obj.TrajectoryHandle, 'top');
            uistack(obj.TrajectoryHandle, 'down');
        end
        
        
        function sendPoseMessage(obj, publisher, pose, n)
            %sendPoseMessage Publish pose message using publisher
            
            x = pose(1);
            y = pose(2);
            theta = pose(3);
            
            % Get position handle and assign values
            position = obj.PoseMessage(n).Pose.Pose.Position;
            position.X = x;
            position.Y = y;
            
            % Convert Euler angles to quaternion
            q = eul2quat([theta 0 0]);
            
            % Get orientation handle and assign values
            orientation = obj.PoseMessage(n).Pose.Pose.Orientation;
            orientation.X = q(2);
            orientation.Y = q(3);
            orientation.Z = q(4);
            orientation.W = q(1);
            
            % Publish velocities as well
            twist = obj.PoseMessage(n).Twist.Twist;
            twist.Linear.X = obj.Robot(n).LinearVelocity;
            twist.Angular.Z = obj.Robot(n).AngularVelocity;
            
            % Publish the pose message
            header = obj.PoseMessage(n).Header;
            header.Seq = header.Seq + 1;
            send(publisher, obj.PoseMessage(n));
        end
        
        function response = resetSimulationServiceCallback(obj, ~, ~, response)
            %resetSimulationServiceCallback Executed when service call to /sim/reset_poses occurs
            
            % Reset the simulation
            obj.resetSimulation;
        end
        
        function response = randomizeLocationServiceCallback(obj, ~, ~, response)
            %randomizeLocationServiceCallback Executed when service call to /sim/new_robot_pose occurs
            
            % Randomize location of robot
            obj.randomizeLocation();
        end
    end
    
    %% All GUI-related methods
    methods (Access = private)
        function randomLocationButtonCallback(obj, ~, ~)
            %randomLocationButtonCallback Callback when user presses "Randomize location" button
            randomizeLocation(obj);
        end
        
        function resetSimulationButtonCallback(obj, ~, ~)
            %resetSimulationButtonCallback Callback when user presses "Reset simulation" button
            resetSimulation(obj);
        end
        
        function cleanup(obj, ~, ~)
            %cleanup Figure window was closed, so delete this object
            delete(obj);
        end
    end
    
end
