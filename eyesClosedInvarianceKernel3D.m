%% Generate Eyes-closed invariance kernel
clear; clc; close all;
save_file = true;

timestamp = now;
compTraj = true;

% Setup Grids
x_min = -10;
y_min = -15;
theta_min = -pi;

x_max = 20;
y_max = 15;
theta_max = pi;

N_x = 71;
N_y = 71;
N_theta = 81;

grid_pursuer_min = [x_min; y_min; theta_min];    % Lower corner of computation domain
grid_pursuer_max = [x_max; y_max; theta_max];    % Upper corner of computation domain

grid_evader_min = [x_min; y_min; theta_min];  
grid_evader_max = [x_max; y_max; theta_max];  

N_pursuer = [N_x; N_y; N_theta];         % Number of grid points per dimension
pdDims_pursuer = 3;                      % 3rd dimension is periodic

N_evader = [N_x; N_y; N_theta];
pdDims_evader = 3;

g_pursuer = createGrid(grid_pursuer_min, grid_pursuer_max, N_pursuer, pdDims_pursuer);
g_evader = createGrid(grid_evader_min, grid_evader_max, N_evader,  pdDims_evader);

%% Generate Initial Set of Pursuer
R = 0.75; % Uncertainty in initial position ((x-x0)^2 + (y-y0)^2 <= R)
W = 0.6; % Uncertainty in initial heading (theta0 +/- W/2)
x0 = [0; 0; 0];
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
cylinder0 = shapeCylinder(g_pursuer, 3, x0, R);
box0 = shapeRectangleByCenter(g_pursuer, x0, [Inf; Inf; W]);
initialPursuerState = shapeIntersection(box0, cylinder0);

%% Time vector for game
t0 = 0;
tMax = 5.0;
dt = 0.10;
tau = t0:dt:tMax;


%% Pursuer params
speed = [0, 3];
wMax = 0.75;

uModePursuer = 'max';
minWithPursuer = 'none';
dCarPursuer = DubinsCarVControl(x0, wMax, speed); % Puruser system

schemeDataPursuer.grid = g_pursuer;
schemeDataPursuer.dynSys = dCarPursuer;
schemeDataPursuer.accuracy = 'high'; %set accuracy
schemeDataPursuer.uMode = uModePursuer;
schemeDataPursuer.tMode = 'forward';

HJIextraArgsPursuer.visualize = true; %show plot
HJIextraArgsPursuer.fig_num = 1; %set figure number
HJIextraArgsPursuer.deleteLastPlot = true; %delete previous plot as you update

%[data, tau, extraOuts] = ...f
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[forward_reach_set, ~, ~] = ...
  HJIPDE_solve(initialPursuerState, tau, schemeDataPursuer, minWithPursuer, HJIextraArgsPursuer);

%% Project and Extrude Forward Reach Set Into Heading and Velocity Dims
radius = 2.0;
extruded_frs = projectExtrudeSet(forward_reach_set, g_pursuer, g_evader, 3, 3, radius);
% extruded_frs = projectExtrudeSet(forward_reach_set, g_pursuer, 3, 3);
%% Modify set to allow for collisions at very low ego velocity. 
% low_vel = 0.25; % Velocity at which colision is acceptable
% high_velocity_region = shapeHyperplane(g_evader,[0,0,-1,0]',[0,0,low_vel,0]');
% negative_velocity_region = shapeHyperplane(g_evader, [0,0,1,0]', [0,0,-0.2,0]');
% 
% for t = 1:length(tau)
%     extruded_frs(:,:,:,:,t) = shapeUnion(shapeIntersection(extruded_frs(:,:,:,:,t), high_velocity_region), negative_velocity_region);
% end
% extruded_frs(:,:,:,:,end) = shapeUnion(high_velocity_region, negative_velocity_region);

%% Evader Parameters

avoidSets = flip(extruded_frs,4);
terminalAvoidSet = avoidSets(:,:,:,1);

x0 = [0;0;0];
% input bounds
speed = [0, 4.0];
wMax = 1.0;

% control trying to min or max value function?
uModeEvader = 'max';
minWithEvader = 'none';

dCarEvader = DubinsCarVControl(x0, wMax, speed); %do dStep3 here

% Put grid and dynamic systems into schemeData
schemeDataEvader.grid = g_evader;
schemeDataEvader.dynSys = dCarEvader;
schemeDataEvader.accuracy = 'high'; %set accuracy
schemeDataEvader.uMode = uModeEvader;

HJIextraArgsEvader.targets = avoidSets;
HJIextraArgsEvader.visualize = true; %show plot
HJIextraArgsEvader.fig_num = 2; %set figure number
HJIextraArgsEvader.deleteLastPlot = true; %delete previous plot as you update
HJIextraArgsEvader.visualizeLevelSet = true;
% HJIextraArgs.visualizeObstaclesFunction = true;
%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[eyes_closed_invariance_kernel, ~, ~] = ...
  HJIPDE_solve(terminalAvoidSet, ...
               tau, ...
               schemeDataEvader, ...
               minWithEvader, ...
               HJIextraArgsEvader);
           
%% Compute optimal trajectory from some initial state
if true
  
  %set the initial state
  xinit = [5, 0, -pi];
  
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g_evader,eyes_closed_invariance_kernel(:,:,:,end),xinit);
  
  if value >= 0 %if initial state is in BES/BET
    % find optimal trajectory
    
    dCarEvader.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = uModeEvader; %set if control wants to min or max 
    TrajextraArgs.visualize = true; %show plot
    TrajextraArgs.fig_num = 3; %figure number
    
    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1 0]; 
    
    %flip data time points so we start from the beginning of time
    dataTraj = flip(eyes_closed_invariance_kernel,4);
    for t = 1:length(tau)
        t
        derivs{t} = computeGradients(g_evader, dataTraj(:,:,:,t));
    end
  else
    error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end
%%
if compTraj && value >= 0
    xinit = [2.5,-1.0,pi/2];
    dCarEvader.x = xinit;
    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [traj, traj_tau] = ...
      computeOptTraj(g_evader, dataTraj, tau, dCarEvader, TrajextraArgs, derivs, flip(avoidSets,4));
end

% %% Process data for saving
% derivs_array = zeros([size(eyes_closed_invariance_kernel),3]);
% for t = 1:length(derivs)
%     for d = 1:3
%         derivs_array(:,:,:,t,d) = derivs{t}{d};
%     end
% end
% 
% eyes_closed_invariance_kernel_orig = eyes_closed_invariance_kernel;
% eyes_closed_invariance_kernel_wrapped = cat(4, ...
%                                             eyes_closed_invariance_kernel, ...
%                                             eyes_closed_invariance_kernel(:,:,:,1,:));
%                                 
% derivs_array = cat(4, derivs_array, derivs_array(:,:,:,1,:,:));
% 
% eyes_closed_invariance_kernel_wrapped = flip(eyes_closed_invariance_kernel_wrapped, 5);
% 
% g_evader_wrapped = g_evader;
% g_evader_wrapped.vs{4} = cat(1, g_evader.vs{4}, g_evader.vs{4}(end) + g_evader.dx(4));
% 
% g_vs_x = g_evader_wrapped.vs{1};
% g_vs_y = g_evader_wrapped.vs{2};
% g_vs_v = g_evader_wrapped.vs{3};
% g_vs_theta = g_evader_wrapped.vs{4};


%% Save data
if false
    save('eyesclosedV1.mat',...
         'eyes_closed_invariance_kernel_wrapped',...
         'derivs_array',...
         'g_vs_x',...
         'g_vs_y',...
         'g_vs_v',...
         'g_vs_theta',...
         'accel',...
         'rMin',...
         'wMax',...
         'tau',...
         'timestamp',...
         '-v7.3')
end
%% Validate lookup
% out = eval_u(g_evader_wrapped, eyes_closed_invariance_kernel_wrapped(:,:,:,:,1), [2.12,-2.21,0.32,2.06])

