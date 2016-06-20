addpath(genpath('~/git/trajgen/matlab'));
clear
close all
clc

robots = {'QuadrotorTango', 'QuadrotorUniform', 'QuadrotorVictor', 'QuadrotorWhiskey'}; %, 'QuadrotorXray'};

% The position gains
traj_kx = [3.7*ones(1,2), 8.0];
traj_kv = [2.4*ones(1,2), 3.0];

% Givens
g = 9.81;
mass = 0.6;
arm_length = 0.175;
prop_r = 0.1;

min_max_rpm = [1200; 5670];
kf = 10.24e-8; % N / rpm^2
km = 0.14 * prop_r * kf;
angular_inertia = 2.6e-3;

% Trajectory generator options
d = 4;
options = {...
    'ndim', d, ...
    'order', 9, ...
    'minderiv', [4,4,4,3], ...
    'constraints_per_seg', 20, ...
    'convergetol', 1e-12, ....
    'numerical', true, ...
    'evalderiv', 4};

for robot = 1:length(robots)
    
    close all
    clear waypoints bounds t
    
    %% Motion along primary axes
    
    z_start = 1.2;
    dt = 1.5;
    
    % Evenly distribute around a circle
    t = 0;
    waypoints(1) = ZeroWaypoint(t, d);
    radius = 1.5;
    waypoints(end).pos = [...
        radius * cos(3*pi/4 - robot / length(robots) * 2 * pi);...
        radius * sin(3*pi/4 - robot / length(robots) * 2 * pi);
        z_start; 0];
    
    home = waypoints(end).pos;
    
    disp(['Motion along primary axes starting at t = ', num2str(waypoints(end).time)]);
    
    % Positive x
    t = t + dt;
    waypoints(end+1) = NanWaypoint(t, d);
    waypoints(end).pos = home + [1; 0; 0; 0];
    
    % Negative x
    t = t + 2*dt;
    waypoints(end+1) = NanWaypoint(t, d);
    waypoints(end).pos = home + [-1; 0; 0; 0];
    
    % Base
    t = t + dt;
    waypoints(end+1) = ZeroWaypoint(t, d);
    waypoints(end).pos = home;
    
    % Positive y
    t = t + dt;
    waypoints(end+1) = NanWaypoint(t, d);
    waypoints(end).pos = home + [0; 0.5; 0; 0];
    
    % Negative y
    t = t + 1.8*dt;
    waypoints(end+1) = NanWaypoint(t, d);
    waypoints(end).pos = home + [0; -0.5; 0; 0];
    
    % Base
    t = t + dt;
    waypoints(end+1) = ZeroWaypoint(t, d);
    waypoints(end).pos = home;
    
    % Positive z
    t = t + 1.5*dt;
    waypoints(end+1) = NanWaypoint(t, d);
    waypoints(end).pos = home + [0; 0; min(3.5 - home(3), z_start + 1); 0];
    % It may be fun use add something like [0.75 * home(1), 0.75 * home(2), ...]
    
    % Negative z to Base
    t = t + 1.5*dt;
    waypoints(end+1) = ZeroWaypoint(t, d);
    waypoints(end).pos = home;
    
    % Positive yaw
    t = t + 3*dt;
    waypoints(end+1) = ZeroWaypoint(t, d);
    waypoints(end).pos = home + [0; 0; 0; 2*pi];
    
    % Negative yaw
    t = t + 3*dt;
    waypoints(end+1) = ZeroWaypoint(t, d);
    waypoints(end).pos = home;
    
    %% Moving around in a helix
    
%     % Delay in starting helix
%     delay = [1; 2; 3; 4];
%     t = t + delay(robot);
%     waypoints(end+1) = ZeroWaypoint(t, d);
%     waypoints(end).pos = home;
    
    z_end = 2.5;
    r_start = 1.5;
    r_end = 1.2;
    t_start_transition = 5;
    start_transition_segments = ceil(t_start_transition/3);
    t_start = t + t_start_transition;
    wo = 0.8;
    w_end = 1.8;
    t_duration = 15;
    segments = ceil(t_duration/3); % 20;
        
    disp(['Transition to Helix starting at t = ', num2str(waypoints(end).time)]);
    for idx = 1:start_transition_segments-1
        waypoints(end+1) = NanWaypoint(waypoints(end).time + t_start_transition/start_transition_segments, d); %#ok<SAGROW>
    end
    
    t_end = t_start + t_duration;
    dt = (t_end - t_start) / segments;
    
    zdot = (z_end - z_start) / (t_end - t_start);
    rdot = (r_end - r_start) / (t_end - t_start);
    % We are assuming rddot, rdddot, and rddddot = 0
    
    wdot = (w_end - wo) / (t_end - t_start);
    % We are assuming wddot, wdddot, and wddddot = 0
    
    yawo = home(4);
    
    disp(['Helix starting at t = ', num2str(t_start)]);

    for total_t = t_start:dt:t_end        

        t = total_t - t_start;
        
        entrance_angle = [pi; pi/2; 0; -pi/2];
        [waypoints(end+1), gamma, gammadot, r, z] = HelixWaypoint(t, total_t, entrance_angle(robot), wo, wdot, yawo, r_start, rdot, home(3), zdot);
        
%         if total_t == t_start + dt
%             p1 = waypoints(end-2).pos;
%             p2 = waypoints(end-1).pos;
%             plot(p1(1), p1(2), 'rx');
%             hold all;
%             plot(p2(1), p2(2), 'gx');
%             xlim([-2.5, 2.5]);
%             ylim([-2.5, 2.5]);
%             keyboard
%         end
    end
    
    %% Continue in a circle
    
    t_start_circ = waypoints(end).time + dt;
    disp(['Continuing in a circle at t = ', num2str(t_start_circ)]);
    t_end = t_start_circ + 7;
    
    % Integrate to the next step
    gamma = gamma + w_end*dt + 1/2 * wdot * dt^2;
    
    % Use a finer resolution for dt
    dt = dt/3;
    
    for total_t = t_start_circ:dt:t_end
        t = total_t - t_start_circ;
        waypoints(end+1) = HelixWaypoint(t, total_t, gamma, gammadot, 0, yawo, r, 0, z, 0);
    end
    
    %% End transition
    
    t_end_transition = 4;
    end_transition_segments = ceil(t_end_transition / 3);
    
    for idx = 1:end_transition_segments-1
        waypoints(end+1) = NanWaypoint(waypoints(end).time + t_end_transition/end_transition_segments, d); %#ok<SAGROW>
    end
    
    waypoints(end+1) = ZeroWaypoint(waypoints(end).time + t_end_transition/end_transition_segments, d);
    waypoints(end).pos = [waypoints(1).pos(1:3)', nan]';
    
%     % Wait for synchronization at end
%     t = waypoints(end).time + max(delay) - delay(robot);
%     waypoints(end+1) = ZeroWaypoint(t, d);
%     waypoints(end).pos = waypoints(end-1).pos;
    
    
    % %% Assuming at base, do something like a criss-cross
    % waypoints(end+1) = NanWaypoint(waypoints(end).time + 2, [home(1:3), waypoints(end).pos(4)]);
    % switch robot
    %   case 1
    %     temp =
    %   case 2
    %   case 3
    %   case 4
    % end
    %     waypoints(end).pos = waypoints(end).pos +
    
    
    bounds = [];
    
    pos = waypoints(end).pos;
    
    %% Figure 8
    
    % t = waypoints(end).time;
    %
    % t = t+1.5;
    % waypoints(end+1) = NanWaypoint(t,d);
    % waypoints(end).pos = [1; 1; pos(3:4)];
    % waypoints(end).vel = [1; 0; 0; 0];
    %
    % t = t+1.5;
    % waypoints(end+1) = NanWaypoint(t,d);
    % waypoints(end).pos = [1; -1; pos(3:4)];
    % waypoints(end).vel = [-1; 0; 0; 0];
    %
    % t = t+2;
    % waypoints(end+1) = NanWaypoint(t,d);
    % waypoints(end).pos = [-1; 1; pos(3:4)];
    % waypoints(end).vel = [-1; 0; 0; 0];
    %
    % t = t+1.5;
    % waypoints(end+1) = NanWaypoint(t,d);
    % waypoints(end).pos = [-1; -1; pos(3:4)];
    % waypoints(end).vel = [1; 0; 0; 0];
    %
    % t = t+1.5;
    % waypoints(end+1) = ZeroWaypoint(t,d);
    % waypoints(end).pos = pos;
    % waypoints(end).vel = nan(4,1);
    %
    % t = t+1.5;
    % waypoints(end+1) = ZeroWaypoint(t,d);
    % waypoints(end).pos = pos;
    
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
    
    tstep = 0.01;
    ntraj = TrajEval(traj, 0:tstep:waypoints(end).time);
    
    plot3(ntraj(:,1,1), ntraj(:,2,1), ntraj(:,3,1));
    hold on
    idxs = round([waypoints.time]/tstep+1);
    h = plot3(ntraj(idxs,1,1), ntraj(idxs,2,1), ntraj(idxs,3,1), 'r.', 'MarkerSize', 10);
    xlabel('x'); ylabel('y'); zlabel('z');
    
    figure
    plot(ntraj(:,4,1));
    xlabel('t')
    ylabel('yaw');
    
    % PlotTraj(traj)
    
    tstep = 0.01;
    ntraj = TrajEval(traj, 0:tstep:waypoints(end).time);
    
    figure();
    quiver(ntraj(:,1,1), ntraj(:,3,1), ntraj(:,1,3), ntraj(:,3,3) + g, 1);
    axis equal
    hold on
    
    idxs = max(1, round([waypoints.time]/tstep));
    plot(ntraj(idxs,1,1), ntraj(idxs,3,1), 'g.', 'MarkerSize', 20);
    
    idxs = max(1, round(t/tstep));
    plot(ntraj(idxs,1,1), ntraj(idxs,3,1), 'r.', 'MarkerSize', 15);
    hold off
        
    %%
    
    % Reevaluate at a finer interval
    ntraj = TrajEval(traj, 0:0.001:waypoints(end).time);
    
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
        ntraj(:,1,1), ntraj(:,2,1), ntraj(:,3,1), ntraj(:,4,1), ...
        ntraj(:,1,2), ntraj(:,2,2), ntraj(:,3,2), ntraj(:,4,2), ...
        ntraj(:,1,3), ntraj(:,2,3), ntraj(:,3,3), ntraj(:,4,3), ...
        ntraj(:,1,4), ntraj(:,2,4), ntraj(:,3,4), ntraj(:,4,4), ...
        gains];
    filename = [robots{robot}, '_traj.csv'];
    
    % Write to file and write dimensions on the first line
    csvwrite(filename, array);
    system(['printf "', num2str(size(array,1)), ',', '4,4,6\n',...
        '$( cat ', filename, ' )" > ', filename]);

    positions{robot} = ntraj(:, 1:3, 1);
    
end

%% Check for collisions

distances = cell(0);
min_distance = [];
for idx = 1:length(robots)-1
    for idx2 = (idx+1):length(robots)
        distances{end+1} = rownorm(positions{idx} - positions{idx2});
        min_distance(end+1) = min(distances{end});
    end
end
plot([distances{:}])

%% Plot the motion

figure()
title('All robots');
for robot = 1:length(robots)
    plot3(positions{robot}(:,1), positions{robot}(:,2), positions{robot}(:,3));
    hold all;
end

poses = [positions{:}];
pos_h = plot3(poses(1, 1:3:10), poses(1, 2:3:11), poses(1, 3:3:12), '.', 'MarkerSize', 30);
step = 50;
for idx = 1:step:size(positions{robot}, 1)
    title(['t = ', num2str(idx / 1000)]);
    set(pos_h, 'XData', poses(idx, 1:3:10));
    set(pos_h, 'YData', poses(idx, 2:3:11));
    set(pos_h, 'ZData', poses(idx, 3:3:12));
    drawnow;
end