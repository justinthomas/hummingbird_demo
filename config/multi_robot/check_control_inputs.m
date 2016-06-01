function flag = check_control_inputs(traj, mass, arm_length, kf, min_max_rpm, angular_inertia, safety, km)
% flag = check_control_inputs(traj, mass, arm_length, kf, min_max_rpm, angular_inertia)

if length(traj) > 1
    keytimes = traj{1}.keytimes;
else
    keytimes = traj.keytimes;
end
tvec = keytimes(1):.01:keytimes(end);
ntraj = TrajEval(traj, tvec);

g = 9.81;

fRe2 = [ntraj(:,1,3), ntraj(:,3,3) + mass * g];
f = rownorm(fRe2);

disp(['Max thrust / weight = ', num2str(max(f)/ (mass * g))]);
b2 = fRe2 ./ [f, f];
b1 = [b2(:,2), -b2(:,1)];

rddd = [ntraj(:,1,4), ntraj(:,3,4)];
fd = sum(b2 .* rddd, 2);
thetad = sum(b1 .* rddd, 2) ./ f;
disp(['Max angular rate = ', num2str(rad2deg(max(thetad))), ' deg/s']);
%flag = rad2deg(max(thetad)) > 500;
flag = false;

rdddd = [ntraj(:,1,5), ntraj(:,3,5)];
thetadd = (sum(b1 .* rdddd,2) - 2 * fd .* thetad) ./ f;

min_rotor_thrust = kf * min_max_rpm(1).^2;
max_rotor_thrust = kf * min_max_rpm(2).^2;
max_ang_acc = arm_length * (max_rotor_thrust - min_rotor_thrust) / angular_inertia;

figure()
plot(tvec, thetadd / max_ang_acc, tvec, abs(f) / (mass * g));
legend('Ratio of max allowed moment', 'thrust/weight');
xlabel('time');
ylabel('value');
drawnow
pause(0.3);
ylim = get(gca, 'YLim');
for idx = 1:length(keytimes)
    line([keytimes(idx), keytimes(idx)], ylim, 'LineStyle', '-.');
end

if min(abs(f) / (mass * g)) < 0.1
    warning('Ahhh');
end

% disp(['Max angular acc = ', num2str(rad2deg(max(thetadd))), ' deg/s^2']);
disp(['Moment required = ', num2str(max(abs(thetadd)) / max_ang_acc), ' * max']);

flag = flag || max(abs(thetadd)) >= safety * max_ang_acc;


% Motor speeds (check if all are real)
figure();
l = arm_length;
ws = sqrt([kf, kf, kf, kf; 0, l*kf, 0 , -l*kf; -l*kf, 0, l*kf, 0; km, -km, km, -km] \ [f'; zeros(1,length(f)); thetadd' * angular_inertia; zeros(1,length(f))]);
if any(any(abs(imag(ws) > 0)))
    error('Solution not realizable');
end
plot(tvec, ws')
line(tvec([1,end]), min_max_rpm([1,1]));
line(tvec([1,end]), min_max_rpm([2,2]));
end

function result = rownorm(mat)
result = sqrt(sum(mat.^2,2));
end
