%% EXAMPLE: Differential Drive Path Following
% In this example, a differential drive robot navigates a set of waypoints 
% using the Pure Pursuit algorithm while avoiding obstacles using the
% Vector Field Histogram (VFH) algorithm.
% 
% Copyright 2019 The MathWorks, Inc.

%% Simulation setup
% Define Vehicle
R = 0.1;                        % Wheel radius [m]
L = 0.5;                        % Wheelbase [m]
dd = DifferentialDrive(R,L);

% Sample time and time array
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:35.5;        % Time array

% Initial conditions
initPose = [1;2;45];            % Initial pose (x y theta)
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;


% Load map

%complexMap       41x52                2132  logical              
%emptyMap         26x27                 702  logical              
%simpleMap        26x27                 702  logical              
%ternaryMap      501x501            2008008  double  

close all
load complexMap

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi,pi,250);%51
lidar.maxRange = 2;%5

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Path planning and following

% Create waypoints
waypoints = [initPose(1:2)'; 
             7  5;
             2 10;
             13 5;
             11 8;
             13 5;
             8 2;
             7 5;
             1 2];

% Pure Pursuit Controller
% Lista de puntos (coordenadas x, y) que el robot debe seguir
controller = controllerPurePursuit;
controller.Waypoints = waypoints;

% Distancia mínima que el robot usa para buscar su siguiente punto objetivo
controller.LookaheadDistance = .5;%0.5

% 	Velocidad lineal deseada para el robot
controller.DesiredLinearVelocity = 1.5; %0.75

%Máxima velocidad angular permitida para girar
controller.MaxAngularVelocity = 20; %20

% Vector Field Histogram (VFH) for obstacle avoidance
% Objeto que arepresenta el controlador VFH para evitar obstáculos.
vfh = controllerVFH;
% Rango mínimo y máximo que considera el sensor para detectar objetos.
vfh.DistanceLimits = [0.05 3]; %0.05 3
% Resolución angular del análisis del entorno.
vfh.NumAngularSectors = 900; %36

% Valores que definen qué tan cerca debe estar un obstáculo para ser considerado.
vfh.HistogramThresholds = [3 8]; % 5y 10
% Radio aproximado del robot, usado para evitar colisiones
vfh.RobotRadius = L*.2;
% Distancia adicional de seguridad respecto a los obstáculos.
vfh.SafetyDistance = L*.2;

% Radio mínimo de giro que puede realizar el robot.
vfh.MinTurningRadius = .1;%0.25


%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
        
    % Run the path following and obstacle avoidance algorithms
    [vRef,wRef,lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 5*steerDir;
    end
    
    % Control the robot
    velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,curPose);  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = curPose + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints,ranges)
    waitfor(r);
end