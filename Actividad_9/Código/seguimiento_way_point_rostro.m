
%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:657;         % Time array
%343
initPose = [6.734200061560114; 11.29938443323006;-45];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Cargar waypoints desde archivo
run("coords.m");
waypoints = waypoints * 2;


% Create visualizer
%viz = Visualizer2D;
%viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = .5;
controller.MaxAngularVelocity = 4.5;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef)
 
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    %viz(pose(:,idx),waypoints)
    %waitfor(r);
    
end

% Mostrar solo la trayectoria final en una figura
figure
plot(pose(1,:), pose(2,:), 'b-', 'LineWidth', 2)  % trayectoria en azul
hold on
plot(waypoints(:,1), waypoints(:,2), 'xr', 'MarkerSize', 6)  % waypoints en rojo
plot(pose(1,1), pose(2,1), 'go', 'MarkerSize', 8, 'LineWidth', 2)  % inicio en verde
plot(pose(1,end), pose(2,end), 'ks', 'MarkerSize', 8, 'LineWidth', 2)  % fin en negro
legend('Trayectoria','Waypoints','Inicio','Fin')
xlabel('X [m]')
ylabel('Y [m]')
title('Trayectoria final del robot')
axis equal
grid on
