%% Generate Eyes-closed invariance kernel
clear; clc; close all;
save_file = true;

timestamp = now;
compTraj = true;

lane_width = 7;

% Setup Grids
x_min = -5;
y_min = -5;
v_min = -0.2;
theta_min = -pi;
alpha_min = -pi;
beta_min = 0;

x_max = 5;
y_max = 5;
v_max = 5.0;
theta_max = pi;
alpha_max = pi;
beta_max = 8;

N_x = 31;
N_y = 31;
N_theta = 16;
N_v = 11;
N_alpha = 21;
N_beta = 21;

grid_pursuer_min = [x_min; y_min; theta_min];    % Lower corner of computation domain
grid_pursuer_max = [x_max; y_max; theta_max];    % Upper corner of computation domain

grid_evader_min = [x_min; y_min; v_min; theta_min; alpha_min; beta_min];  
grid_evader_max = [x_max; y_max; v_max; theta_max; alpha_max; beta_max];  

N_pursuer = [N_x; N_y; N_theta];         % Number of grid points per dimension
pdDims_pursuer = 3;                      % 3rd dimension is periodic

N_evader = [N_x; N_y; N_v; N_theta; N_alpha; N_beta];
pdDims_evader = [4, 5];

g_pursuer = createGrid(grid_pursuer_min, grid_pursuer_max, N_pursuer, pdDims_pursuer);
g_evader = createGrid(grid_evader_min, grid_evader_max, N_evader,  pdDims_evader);

%% Time vector for game
t0 = 0;
tMax = 1.0;
dt = 0.20;
tau = t0:dt:tMax;

%% Generate Initial Set of Pursuer
R = 0.75; % Uncertainty in initial position ((x-x0)^2 + (y-y0)^2 <= R)
W = 0.5; % Uncertainty in initial heading (theta0 +/- W/2)
x0 = [0; 0; 0];
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
cylinder0 = shapeCylinder(g_pursuer, 3, x0, R);
box0 = shapeRectangleByCenter(g_pursuer, x0, [Inf; Inf; W]);
initialPursuerState = shapeIntersection(box0, cylinder0);




%% Pursuer params
speed = [0, 2];
wMax = 1;

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

%% Project Forward Reach Set in heading dim, and extrude back in heading, velocity, alpha, and beta dims. 
extruded_frs = projectExtrudeSet(forward_reach_set, g_evader, 3, [3,4,5,6]);

%% Modify set to allow for collisions at very low ego velocity. 
low_vel = 0.5; % Velocity at which colision is acceptable
high_velocity_region = shapeHyperplane(g_evader,[0,0,-1,0,0,0]',[0,0,low_vel,0,0,0]');
negative_velocity_region = shapeHyperplane(g_evader, [0,0,1,0,0,0]', [0,0,-0.2,0,0,0]');

for t = 1:length(tau)
    extruded_frs(:,:,:,:,:,:,t) = shapeUnion(shapeIntersection(extruded_frs(:,:,:,:,:,:,t), high_velocity_region), negative_velocity_region);
end
extruded_frs(:,:,:,:,:,:,end) = shapeUnion(high_velocity_region, negative_velocity_region);

%% Modify set to avoid lane line boundaries
lane_boundaries = -shapeLaneBoundaries(g_evader,1,2,5,6,lane_width);
% Inside lane boundaires is positive. Outside is negative

extruded_frs_lane = extruded_frs;
for t = 1:length(tau)
    extruded_frs_lane(:,:,:,:,:,:,t) = shapeUnion(lane_boundaries, extruded_frs(:,:,:,:,:,:,t));
end


%% Evader Parameter

avoidSets = flip(extruded_frs,7);
terminalAvoidSet = avoidSets(:,:,:,:,:,:,1); % Inside set is negative. Outside set is positive

x0 = [0;0;0;0;0;0];
% input bounds
accel = [-1.5, 1.5];
wMax = 1;

% control trying to min or max value function?
uModeEvader = 'max';
minWithEvader = 'none';

dCarEvader = DubinsCarAControlGlobalPos(x0, wMax, accel); %do dStep3 here

% Put grid and dynamic systems into schemeData
schemeDataEvader.grid = g_evader;
schemeDataEvader.dynSys = dCarEvader;
schemeDataEvader.accuracy = 'high'; %set accuracy
schemeDataEvader.uMode = uModeEvader;

HJIextraArgsEvader.targets = avoidSets;
HJIextraArgsEvader.visualize = false; %show plot
HJIextraArgsEvader.fig_num = 2; %set figure number
HJIextraArgsEvader.deleteLastPlot = true; %delete previous plot as you update
HJIextraArgsEvader.visualizeLevelSet = false;
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
if compTraj
  
  %set the initial state
  xinit = [-4, 0, 0, pi, pi/2, 0];
  
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g_evader,eyes_closed_invariance_kernel(:,:,:,:,:,:,end),xinit);
  
  if value >= 0 %if initial state is in BES/BET
    % find optimal trajectory
    
    dCarEvader.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = uModeEvader; %set if control wants to min or max 
    TrajextraArgs.visualize = true; %show plot
    TrajextraArgs.fig_num = 3; %figure number
    
    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1 0 0 0 0]; 
    
    %flip data time points so we start from the beginning of time
    dataTraj = flip(eyes_closed_invariance_kernel,7);
    for t = 1:length(tau)
        t
        derivs{t} = computeGradients(g_evader, dataTraj(:,:,:,:,:,:,t));
    end
  else
    error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end
%%
compTraj = true;
if compTraj && value >= 0
    xinit = [3, 0, 2, -pi, pi/2, 0];
    dCarEvader.x = xinit;
    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [traj, traj_tau] = ...
      computeOptTraj(g_evader, dataTraj, tau, dCarEvader, TrajextraArgs, derivs, flip(avoidSets,7));
end

%% Process data for saving
derivs_array = zeros([size(eyes_closed_invariance_kernel),4]);
for t = 1:length(derivs)
    for d = 1:4
        derivs_array(:,:,:,:,t,d) = derivs{t}{d};
    end
end

eyes_closed_invariance_kernel_orig = eyes_closed_invariance_kernel;
eyes_closed_invariance_kernel_wrapped = cat(4, ...
                                            eyes_closed_invariance_kernel, ...
                                            eyes_closed_invariance_kernel(:,:,:,1,:));
                                
derivs_array = cat(4, derivs_array, derivs_array(:,:,:,1,:,:));

eyes_closed_invariance_kernel_wrapped = flip(eyes_closed_invariance_kernel_wrapped, 5);

g_evader_wrapped = g_evader;
g_evader_wrapped.vs{4} = cat(1, g_evader.vs{4}, g_evader.vs{4}(end) + g_evader.dx(4));

g_vs_x = g_evader_wrapped.vs{1};
g_vs_y = g_evader_wrapped.vs{2};
g_vs_v = g_evader_wrapped.vs{3};
g_vs_theta = g_evader_wrapped.vs{4};


%% Save data
if false
    save('eyesclosedV0.mat',...
         'eyes_closed_invariance_kernel_wrapped',...
         'derivs_array',...
         'g_vs_x',...
         'g_vs_y',...
         'g_vs_v',...
         'g_vs_theta',...
         'accel',...
         'wMax',...
         'tau',...
         'timestamp',...
         '-v7.3')
end
%% Validate lookup
out = eval_u(g_evader_wrapped, eyes_closed_invariance_kernel_wrapped(:,:,:,:,1), [2.12,-2.21,0.32,2.06])

