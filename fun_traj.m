addpath(genpath('~/git/trajgen/matlab'));
clear
close all
clc

% The position gains
traj_kx = [3.7*ones(1,2), 8.0];
traj_kv = [2.4*ones(1,2), 3.0];

% Givens
g = 9.81;
mass = 0.5;
arm_length = 0.175;
prop_r = 0.1;

min_max_rpm = [1200; 5670];
kf = 10.24e-8; % N / rpm^2
km = 0.14 * prop_r * kf;
angular_inertia = 2.6e-3;

% Trajectory generator options
d = 4;
options = {'ndim',d ,'order',9, 'minderiv',[4,4,4,3], 'constraints_per_seg', 20, 'convergetol', 1e-9};

%% Moving around in a helix

close all
clear waypoints bounds

z_start = 0.5;
z_end = 1.5;
r_start = 1.5;
r_end = 0.75;
t_start = 5;
t_end_transition = 3;
wo = 1;
w_end = 3.75;
t_duration = 30;
segments = 30;
start_transition_segments = 1;
end_transition_segments = 5;

t = 0;
waypoints(1) = ZeroWaypoint(t, d);
waypoints(end).pos = [0; 0; z_start; 0];

for idx = 1:start_transition_segments-1
    waypoints(end+1) = NanWaypoint(t_start/start_transition_segments, d); %#ok<SAGROW>
    waypoints(end).pos(end) = 0;
end

t_end = t_start + t_duration;
dt = (t_end - t_start) / segments;

zdot = (z_end - z_start) / (t_end - t_start);
rdot = (r_end - r_start) / (t_end - t_start);
% We are assuming rddot, rdddot, and rddddot = 0

wdot = (w_end - wo) / (t_end - t_start);
% We are assuming wddot, wdddot, and wddddot = 0

for total_t = t_start:dt:t_end
    
    waypoints(end+1) = NanWaypoint(total_t, d); %#ok<SAGROW>
    
    t = total_t - t_start;
    
    r = r_start + rdot * t;
    w = wo + wdot * t;
    
    yaw = - (wo * t + 1/2 * wdot * t^2);
    yaw_dot = - (wo + wdot * t);
    yaw_ddot = - wdot;

    waypoints(end).pos = [...
        r * cos(-yaw);...
        r * sin(-yaw);...
        z_start + t*zdot;...
        yaw]; % This isn't correct
 
    waypoints(end).vel = [...
        cos(1/2*t*(2*wo+t*(wdot)))*(rdot)-r*sin(1/2*t*(2*wo+t*(wdot)))*(wo+t*(wdot));...
        sin(1/2*t*(2*wo+t*(wdot)))*(rdot)+cos(1/2*t*(2*wo+t*(wdot)))*r*(wo+t*(wdot));...
        zdot;...
        yaw_dot];
    
    waypoints(end).acc = [...
        -cos(1/2*t*(2*wo+t*(wdot)))*r*(wo+t*(wdot)).^2-sin(1/2*t*(2*wo+t*(wdot)))*(r*(wdot)+2*(rdot)*(wo+t*(wdot)));...
        -r*sin(1/2*t*(2*wo+t*(wdot)))*(wo+t*(wdot)).^2+cos(1/2*t*(2*wo+t*(wdot)))*(r*(wdot)+2*(rdot)*(wo+t*(wdot)));...
        0; yaw_ddot];
    
    waypoints(end).jerk = [...
        -3*cos(1/2*t*(2*wo+t*(wdot)))*(wo+t*(wdot))*(r*(wdot)+(rdot)*(wo+t*(wdot)))+sin(1/2*t*(2*wo+t*(wdot)))*(-3*(rdot)*(wdot)+r*(wo+t*(wdot)).^3);...
        -3*sin(1/2*t*(2*wo+t*(wdot)))*(wo+t*(wdot))*(r*(wdot)+(rdot)*(wo+t*(wdot)))-cos(1/2*t*(2*wo+t*(wdot)))*(-3*(rdot)*(wdot)+r*(wo+t*(wdot)).^3);...
        0; nan];
    
    waypoints(end).snap = [...
        2*sin(1/2*t*(2*wo+t*(wdot)))*(wo+t*(wdot)).^2*(3*r*(wdot)+2*(rdot)*(wo+t*(wdot)))+cos(1/2*t*(2*wo+t*(wdot)))*(-12*(rdot)*(wdot)*(wo+t*(wdot))+r*(wo.^4+(wdot)*(4*t*wo.^3+(wdot)*(-3+6*t.^2*wo.^2+t.^3*(wdot)*(4*wo+t*(wdot))))));...
        -2*cos(1/2*t*(2*wo+t*(wdot)))*(wo+t*(wdot)).^2*(3*r*(wdot)+2*(rdot)*(wo+t*(wdot)))+sin(1/2*t*(2*wo+t*(wdot)))*(-12*(rdot)*(wdot)*(wo+t*(wdot))+r*(wo.^4+(wdot)*(4*t*wo.^3+(wdot)*(-3+6*t.^2*wo.^2+t.^3*(wdot)*(4*wo+t*(wdot))))));...
        0; nan];
end

% pos = waypoints(end).pos;
% size(waypoints)

%% 
% % Just keep going in a circle
% t_start_circ = total_t + dt;
% t_end = t_start_circ + 2;
% 
% z = pos(3);
% yaw_start = pos(4) - w_end * dt;
% for total_t = t_start_circ:dt:t_end
%     
%     waypoints(end+1) = NanWaypoint(total_t, d); %#ok<SAGROW>
%     
%     t = total_t - t_start_circ;
%     
%     wo = w_end;
%     r = r_end;
%     wdot = 0;
%     rdot = 0;
%     
%     yaw = yaw_start - wo * t;
%     yaw_dot = - w_end;
%     yaw_ddot = 0;
%     
%     pos_yaw = -yaw;
%     
%     waypoints(end).pos = [...
%         r * cos(pos_yaw);...
%         r * sin(pos_yaw);...
%         z;...
%         yaw]; % This isn't correct
%  
%     waypoints(end).vel = [...
%         -wo*r*sin(pos_yaw);...
%         wo*cos(pos_yaw)*r;...
%         0;...
%         yaw_dot];
%     
%     waypoints(end).acc = [...
%         -wo.^2*cos(pos_yaw)*r;...
%         -wo.^2*r*sin(pos_yaw);...
%          0; 0];
%     
%     waypoints(end).jerk = [...
%         wo.^3*r*sin(pos_yaw);...
%         -wo.^3*cos(pos_yaw)*r;...
%         0; nan];
%     
%     waypoints(end).snap = [...
%         wo.^4*cos(pos_yaw)*r;...
%         wo.^4*r*sin(pos_yaw);...
%         0; nan];
%     
% end
% 
% size(waypoints)

for idx = 1:end_transition_segments-1
    waypoints(end+1) = NanWaypoint(waypoints(end).time + t_end_transition/end_transition_segments, d); %#ok<SAGROW>
end

waypoints(end+1) = ZeroWaypoint(waypoints(end).time + t_end_transition/end_transition_segments, d);
waypoints(end).pos = [waypoints(1).pos(1:3)', nan]';

bounds = [];
% bounds(1) = SetBound([], 'pos', 'lb', [nan, nan, -10, nan]);
% bounds(1) = SetBound([], 'acc', 'ub', [1.0 * g, nan, 1.7 * g - g, nan]);
% bounds(end+1) = SetBound([], 'acc', 'lb', [-1.0 * g, nan, - g, nan]);
% 
% bounds(end+1) = SetBound([], 'jerk', 'ub', [ones(1,3) * 100, nan]);
% bounds(end+1) = SetBound([], 'jerk', 'lb', [ones(1,3) * -100, nan]);
% 
% bounds(end+1) = SetBound([], 'snap', 'ub', [ones(1,3) * 2000, nan]);
% bounds(end+1) = SetBound([], 'snap', 'lb', [ones(1,3) * -2000, nan]);

pos = waypoints(end).pos;

%% Figure 8

t = waypoints(end).time;

t = t+1.5;
waypoints(end+1) = NanWaypoint(t,d);
waypoints(end).pos = [1; 1; pos(3:4)];
waypoints(end).vel = [1; 0; 0; 0];

t = t+1.5;
waypoints(end+1) = NanWaypoint(t,d);
waypoints(end).pos = [1; -1; pos(3:4)];
waypoints(end).vel = [-1; 0; 0; 0];

t = t+2;
waypoints(end+1) = NanWaypoint(t,d);
waypoints(end).pos = [-1; 1; pos(3:4)];
waypoints(end).vel = [-1; 0; 0; 0];

t = t+1.5;
waypoints(end+1) = NanWaypoint(t,d);
waypoints(end).pos = [-1; -1; pos(3:4)];
waypoints(end).vel = [1; 0; 0; 0];

t = t+1.5;
waypoints(end+1) = ZeroWaypoint(t,d);
waypoints(end).pos = pos;
waypoints(end).vel = nan(4,1);

%% Fake Falling

% t_transition = t+1:1:t+3;
% for idx = 1:length(t_transition)
%     t = t_transition(idx);
%     waypoints(end+1) = NanWaypoint(t,d); %#ok<SAGROW>
% end
% waypoints(end).pos = [nan, nan, 1.7, pos(4)];
% waypoints(end).vel = [0, 0, -2.3, 0];
% % waypoints(end).acc = [nan, nan, 0, nan];
% % waypoints(end).jerk = [nan, nan, 0, nan];
% 
% ts = t+1:1:t+2;
% for idx = 1:length(ts)
%     t = ts(idx);
%     waypoints(end+1) = NanWaypoint(t,d); %#ok<SAGROW>
% end
% 
% t = t+1;
% waypoints(end+1) = ZeroWaypoint(t,d);
% waypoints(end).pos = [0, 0, 1, pos(4)];

%% Generate the trajectory

traj = trajgen(waypoints, options, bounds);

check_control_inputs(traj, mass, arm_length, kf, min_max_rpm, angular_inertia, 1, km);

tstep = 0.01;
ntraj = TrajEval(traj, 0:tstep:traj.keytimes(end));

plot3(ntraj(:,1,1), ntraj(:,2,1), ntraj(:,3,1));
hold on
idxs = round([waypoints.time]/tstep+1);
h = plot3(ntraj(idxs,1,1), ntraj(idxs,2,1), ntraj(idxs,3,1), 'r.');
xlabel('x'); ylabel('y'); zlabel('z');

figure
plot(ntraj(:,4,1));
xlabel('t')
ylabel('yaw');

%PlotTraj(traj)

%% Figure 8

%% Falling leaf like crashing and stops right before ground

%% Loop
% %
% % Note that we only need to plan for x and z since the other dimensions are
% % identically zero
% 
% close all;
% clear waypoints traj ntraj bounds;
% 
% options = {'ndim',d ,'order',11, 'minderiv',4*ones(1,d), 'constraints_per_seg', 20, 'convergetol', 1e-8, 'contderiv', 5*ones(1,d)};
% 
% t = [];
% t_top = 1.8;
% divs = 8;
% 
% % Trajectory Start
% waypoints(1) = ZeroWaypoint(0, d);
% waypoints(end).pos = [-1; 0; 1.5];
% waypoints(end).vel = [0; 0; 0];
% 
% % Some extra DOFs
% for idx = 1:divs-1
%     waypoints(end+1) = NanWaypoint(idx * t_top/divs, d); %#ok<SAGROW>
% end
% 
% % Top of loop
% waypoints(end+1) = NanWaypoint(t_top, d);
% waypoints(end).pos = [0; 0; 3.8];
% % waypoints(end).vel = [2.5; 0; 0];
% waypoints(end).acc = [0; 0; nan];
% bounds(1) = SetBound(t_top, 'acc', 'ub', [nan, nan, -1.4*g]);
% bounds(end+1) = SetBound(t_top, 'vel', 'lb', [3.0, nan, nan]);
% 
% % Some extra DOFs
% for idx = 1:divs-1
%     waypoints(end+1) = NanWaypoint(t_top + idx * t_top/divs, d); %#ok<SAGROW>
% end
% 
% % Bottom of loop
% waypoints(end+1) = ZeroWaypoint(2*t_top, d);
% waypoints(end).pos = waypoints(1).pos .* [-1; 1; 1];
% 
% % Duration Bounds
% 
% bounds(end+1) = SetBound([], 'pos', 'lb', [nan, nan, waypoints(1).pos(3) - 0.3]);
% 
% bounds(end+1) = SetBound([], 'acc', 'ub', [1.2*g, nan, 1.5 * g - g]);
% bounds(end+1) = SetBound([], 'acc', 'lb', [-1.2*g, nan, -1.7 * g - g]);
% 
% bounds(end+1) = SetBound([], 'jerk', 'ub', ones(1,3) * 35);
% bounds(end+1) = SetBound([], 'jerk', 'lb', ones(1,3) * -35);
% 
% bounds(end+1) = SetBound([], 'snap', 'ub', ones(1,3) * 2000);
% bounds(end+1) = SetBound([], 'snap', 'lb', ones(1,3) * -2000);
% 
% % Generate the trajectory
% traj = trajgen(waypoints, options, bounds);
% 
% %Checking the trajectory
% check_control_inputs(traj, mass, arm_length, kf, min_max_rpm, angular_inertia, 1, km);
% 
% tstep = t_top / divs

%% Plotting

% PlotTraj(traj)

tstep = 0.01;
ntraj = TrajEval(traj, 0:tstep:traj.keytimes(end));

figure();
quiver(ntraj(:,1,1), ntraj(:,3,1), ntraj(:,1,3), ntraj(:,3,3) + g, 1);
axis equal
hold on

idxs = max(1, round(traj.keytimes/tstep));
plot(ntraj(idxs,1,1), ntraj(idxs,3,1), 'g.', 'MarkerSize', 20);

idxs = max(1, round(t/tstep));
plot(ntraj(idxs,1,1), ntraj(idxs,3,1), 'r.', 'MarkerSize', 15);
hold off

%%

% Reevaluate at a finer interval
ntraj = TrajEval(traj, 0:0.001:traj.keytimes(end));

% Some safety checks
max_acc = max(sqrt(ntraj(:,1,3).^2 + ntraj(:,2,3).^2));
% fprintf('Max Acceleration: %2.2f m/s\n', max_acc);

%% Compute gains

% close all; plot(smooth(ntraj(:,3,3)>-0.4*g,250),'r'); hold on; plot(ntraj(:,3,3)>-g);

bools = (ntraj(:,3,3) > 0.5*g)*0;
bools(1:900) = 1;
bools(end-900:end) = 1;
multiplier = ones(size(ntraj(:,3,3))); %smooth(bools, 1000, 'lowess');

gains = [bsxfun(@times, multiplier, traj_kx), ...
         bsxfun(@times, multiplier, traj_kv)];

% figure();
% plot(gains);

%% Output to a csv

% Generate the array
array = [...
    ntraj(:,2,1), ntraj(:,1,1), ntraj(:,3,1), ntraj(:,4,1), ...
    ntraj(:,2,2), ntraj(:,1,2), ntraj(:,3,2), ntraj(:,4,2), ...
    ntraj(:,2,3), ntraj(:,1,3), ntraj(:,3,3), ntraj(:,4,3), ...
    ntraj(:,2,4), ntraj(:,1,4), ntraj(:,3,4), ntraj(:,4,4), ...
    gains];
filename = 'traj.csv';

% Write to file and write dimensions on the first line
csvwrite(filename, array);
system(['printf "', num2str(size(array,1)), ',', '4,4,6\n',...
    '$( cat ', filename, ' )" > ', filename]);
