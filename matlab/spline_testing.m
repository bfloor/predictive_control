close all
clear all
clc

addpath(genpath('ebertolazzi-G1fitting-04d0af0/'))
addpath(genpath('interparc/'))

%% Define some constants

global PLOT_FLAG
PLOT_FLAG = true;

% Traveled distance (usually integrated by MPCC solver as a state)
s = 0;

% Number of spline points
n_points_spline = 10;
n_points_interpolate = 100;

% Number of spline points which are given to the solver
global N_SPLINE_POINTS;
N_SPLINE_POINTS = 30;

% Define road boundaries
left_boundary = 1;
right_boundary = 1;

% prediction horizon of FORCES MPCC controller
N = 50;

% Number of MPCC parameters
n_other_param = 500;
n_spline_param = 1000;
npar =  n_other_param + n_spline_param;          % total number of parameters

%% Define spline crossing points and corresponding headings

x = [ 0, 10, 10];
y = [ 0, 0,  10];
theta = [0, pi/2, pi/2];

%% Compute and visualize spline

% https://nl.mathworks.com/matlabcentral/fileexchange/42113-ebertolazzi-g1fitting
% https://github.com/ozymandium/g1fitting
[S_road, dist_spline_pts] = get_spline( x, y, theta, n_points_spline );

%% Interpolate generated spline

road_path = ppval(S_road, linspace(0,(n_points_spline-1)*dist_spline_pts,n_points_interpolate));
j=1;
for i=0:(n_points_spline-1)*dist_spline_pts:n_points_interpolate
    road_path(1,j) = S_road.coefs(1)*i^3 + S_road.coefs(2)*i^2 + S_road.coefs(3)*i + S_road.coefs(4);
    road_path(2,j) = S_road.coefs(1)*i^3 + S_road.coefs(2)*i^2 + S_road.coefs(3)*i + S_road.coefs(4);
    j=j+1;
end

M = diag(3:-1:1,1);
S_ds_road = S_road;
S_ds_road.coefs = S_ds_road.coefs*M;
road_path_ds = ppval(S_ds_road, linspace(0,(n_points_spline-1)*dist_spline_pts,n_points_interpolate));

%% Compute path gradients and road boundaries -- currently not used

dx_path_norm = cos( atan2( road_path_ds(2,:), road_path_ds(1,:)));  
dy_path_norm = sin( atan2( road_path_ds(2,:), road_path_ds(1,:)));

boundary1 = road_path + left_boundary*[-dy_path_norm; dx_path_norm];        
boundary2 = road_path - right_boundary*[-dy_path_norm; dx_path_norm];

%% Visualize interpolated path with boundaries

if PLOT_FLAG
    figure
    plot(road_path(1,:),road_path(2,:))
    hold on
    plot(boundary1(1,:),boundary1(2,:), 'r' )
    plot(boundary2(1,:),boundary2(2,:), 'r' )
    title('Intepolated path with road boundaries')
end
