
%% plot CoM desired, measured and error

time          = comData.time;

% Desired CoM
com_desired_x = comData.signals(2).values(1,:,:);
com_desired_x = reshape(com_desired_x, size(com_desired_x,3),1);

com_desired_y = comData.signals(2).values(2,:,:);
com_desired_y = reshape(com_desired_y, size(com_desired_y,3),1);

com_desired_z = comData.signals(2).values(3,:,:);
com_desired_z = reshape(com_desired_z, size(com_desired_z,3),1);

% Measued CoM
com_measured_x = comData.signals(1).values(1,:,:);
com_measured_x = reshape(com_measured_x, size(com_measured_x,3),1);

com_measured_y = comData.signals(1).values(2,:,:);
com_measured_y = reshape(com_measured_y, size(com_measured_y,3),1);

com_measured_z = comData.signals(1).values(3,:,:);
com_measured_z = reshape(com_measured_z, size(com_measured_z,3),1);

% Error =  Desired CoM - Measured CoM
com_error_x   =  com_measured_x - com_desired_x;
com_error_y   =  com_measured_y - com_desired_y;
com_error_z   =  com_measured_z - com_desired_z;

ax1 = subplot(3,1,1); % CoM desired subplot
plot(time, com_desired_x, 'r', time, com_desired_y, 'g', time, com_desired_z, 'b')
title(ax1,'CoM Desired')
legend('x','y','z')
grid on

ax2 = subplot(3,1,2); %CoM measured subplot
plot(time, com_measured_x, 'r', time, com_measured_y, 'g', time, com_measured_z, 'b')
title(ax2,'CoM Measured')
legend('x','y','z')
grid on


ax3 = subplot(3,1,3); %CoM error subplot
plot(time, com_error_x, 'r', time, com_error_y, 'g', time, com_error_z, 'b')
title(ax3,'CoM Error')
legend('x','y','z')
grid on

%% plot contact wrenches desired, estimated, error

% desired left foot wrenches
left_F_x = FORCES.signals(5).values(:,1);

left_F_y = FORCES.signals(5).values(:,2);

left_F_z = FORCES.signals(5).values(:,3);

left_M_x = FORCES.signals(5).values(:,4);

left_M_y = FORCES.signals(5).values(:,5);

left_M_z = FORCES.signals(5).values(:,6);

% estimated left foot wrenches
left_F_x_estimated = FORCES.signals(1).values(:,1);

left_F_y_estimated = FORCES.signals(1).values(:,2);

left_F_z_estimated = FORCES.signals(1).values(:,3);

left_M_x_estimated = FORCES.signals(1).values(:,4);

left_M_y_estimated = FORCES.signals(1).values(:,5);

left_M_z_estimated = FORCES.signals(1).values(:,6);

% Error between estimated and esired left contact wrenches
left_F_x_error = FORCES.signals(3).values(:,1);

left_F_y_error = FORCES.signals(3).values(:,2);

left_F_z_error = FORCES.signals(3).values(:,3);

left_M_x_error = FORCES.signals(3).values(:,4);

left_M_y_error = FORCES.signals(3).values(:,5);

left_M_z_error = FORCES.signals(3).values(:,6);


% desired right foot wrenches
right_F_x = FORCES.signals(6).values(:,1);

right_F_y = FORCES.signals(6).values(:,2);

right_F_z = FORCES.signals(6).values(:,3);

right_M_x = FORCES.signals(6).values(:,4);

right_M_y = FORCES.signals(6).values(:,5);

right_M_z = FORCES.signals(5).values(:,6);

% estimated right foot wrenches
right_F_x_estimated = FORCES.signals(2).values(:,1);

right_F_y_estimated = FORCES.signals(2).values(:,2);

right_F_z_estimated = FORCES.signals(2).values(:,3);

right_M_x_estimated = FORCES.signals(2).values(:,4);

right_M_y_estimated = FORCES.signals(2).values(:,5);

right_M_z_estimated = FORCES.signals(2).values(:,6);

% Error between estimated and esired right contact wrenches
right_F_x_error = FORCES.signals(4).values(:,1);

right_F_y_error = FORCES.signals(4).values(:,2);

right_F_z_error = FORCES.signals(4).values(:,3);

right_M_x_error = FORCES.signals(4).values(:,4);

right_M_y_error = FORCES.signals(4).values(:,5);

right_M_z_error = FORCES.signals(4).values(:,6);


figure
ax1 = subplot(6,1,1); % left desired wrenches subplot
plot(time, left_F_x, time, left_F_y, time, left_F_z, time, left_M_x, time, left_M_y, time, left_M_z)
title(ax1,'Desired contact wrenches left foot')
legend('F_x','F_y','F_z', 'M_x', 'M_y', 'M_z')
grid on

ax2 = subplot(6,1,2); % left estimated wrenches subplot
plot(time, left_F_x_estimated, time, left_F_y_estimated, time, left_F_z_estimated, time, left_M_x_estimated, time, left_M_y_estimated, time, left_M_z_estimated)
title(ax2,'Estimated contact wrenches left foot')
legend('F_x','F_y','F_z', 'M_x', 'M_y', 'M_z')
grid on

ax3 = subplot(6,1,3); % left wrenches error subplot
plot(time, left_F_x_error, time, left_F_y_error, time, left_F_z_error, time, left_M_x_error, time, left_M_y_error, time, left_M_z_error)
title(ax3,'Error contact wrenches left foot')
legend('F_x','F_y','F_z', 'M_x', 'M_y', 'M_z')
grid on

ax4 = subplot(6,1,4); % right desired wrenches subplot
plot(time, right_F_x, time, right_F_y, time, right_F_z, time, right_M_x, time, right_M_y, time, right_M_z)
title(ax4,'Desired contact wrenches right foot')
legend('F_x','F_y','F_z', 'M_x', 'M_y', 'M_z')
grid on

ax5 = subplot(6,1,5); % right estimated wrenches subplot
plot(time, right_F_x_estimated, time, right_F_y_estimated, time, right_F_z_estimated, time, right_M_x_estimated, time, right_M_y_estimated, time, right_M_z_estimated)
title(ax5,'Estimated contact wrenches right foot')
legend('F_x','F_y','F_z', 'M_x', 'M_y', 'M_z')
grid on

ax6 = subplot(6,1,6); % right wrenches error subplot
plot(time, right_F_x_error, time, right_F_y_error, time, right_F_z_error, time, right_M_x_error, time, right_M_y_error, time, right_M_z_error)
title(ax6,'Error contact wrenches right foot')
legend('F_x','F_y','F_z', 'M_x', 'M_y', 'M_z')
grid on

%% plot linear and angular momentum errors
time                    = error_H_linear.time;

% linear momentum error
linear_momentum_error_x = error_H_linear.signals(1).values(:,1);
linear_momentum_error_y = error_H_linear.signals(1).values(:,2);
linear_momentum_error_z = error_H_linear.signals(1).values(:,3);

% angular momentum error
angular_momentum_error_x = error_H_angular.signals(1).values(:,1);
angular_momentum_error_y = error_H_angular.signals(1).values(:,2);
angular_momentum_error_z = error_H_angular.signals(1).values(:,3);

figure
ax1 = subplot(2,1,1); % linear momentum errors subplot
plot(time, linear_momentum_error_x, time, linear_momentum_error_y, time, linear_momentum_error_z)
title(ax1,'Linear momentum error')
legend('x','y','z')
grid on

ax2 = subplot(2,1,2); % angular momentum errors subplot
plot(time, angular_momentum_error_x, time, angular_momentum_error_y, time, angular_momentum_error_z)
title(ax2,'angular momentum error')
legend('x','y','z')
grid on

%% plot 
time              = jointErrorData.time;

% Torso joint errors
torso_roll_error  = jointErrorData.signals(1).values(:,1);
torso_pitch_error = jointErrorData.signals(1).values(:,2);
torso_yaw_error   = jointErrorData.signals(1).values(:,3);

% left arm joint errors
left_shoulder_roll_error  = jointErrorData.signals(2).values(:,1);
left_shoulder_pitch_error = jointErrorData.signals(2).values(:,2);
left_shoulder_yaw_error   = jointErrorData.signals(2).values(:,3);
left_elbow_error          = jointErrorData.signals(2).values(:,4);

% right arm joint errors
right_shoulder_roll_error  = jointErrorData.signals(3).values(:,1);
right_shoulder_pitch_error = jointErrorData.signals(3).values(:,2);
right_shoulder_yaw_error   = jointErrorData.signals(3).values(:,3);
right_elbow_error          = jointErrorData.signals(3).values(:,4);

% left leg joint errors
left_hip_roll_error        = jointErrorData.signals(4).values(:,1);
left_hip_pitch_error       = jointErrorData.signals(4).values(:,2);
left_hip_yaw_error         = jointErrorData.signals(4).values(:,3);
left_knee_error            = jointErrorData.signals(4).values(:,4);
left_ankle_roll_error      = jointErrorData.signals(4).values(:,5);
left_ankle_pitch_error     = jointErrorData.signals(4).values(:,6);

% right leg joint errors
right_hip_roll_error       = jointErrorData.signals(5).values(:,1);
right_hip_pitch_error      = jointErrorData.signals(5).values(:,2);
right_hip_yaw_error        = jointErrorData.signals(5).values(:,3);
right_knee_error           = jointErrorData.signals(5).values(:,4);
right_ankle_roll_error     = jointErrorData.signals(5).values(:,5);
right_ankle_pitch_error    = jointErrorData.signals(5).values(:,6);


figure
ax1 = subplot(5,1,1); % Torso joint errors subplot
plot(time, torso_roll_error, time, torso_pitch_error, time, torso_yaw_error)
title(ax1,'Torso joint errors')
legend('roll','pitch','yaw')
grid on

ax2 = subplot(5,1,2); % left arm joint errors subplot
plot(time, left_shoulder_roll_error, time, left_shoulder_pitch_error, time, left_shoulder_yaw_error, time, left_elbow_error)
title(ax2,'Left arm joint errors')
legend('shoulder_roll','shoulder_pitch','shoulder_yaw', 'shoulder_elbow')
grid on

ax3 = subplot(5,1,3); % right arm joint errors subplot
plot(time, right_shoulder_roll_error, time, right_shoulder_pitch_error, time, right_shoulder_yaw_error, time, right_elbow_error)
title(ax3,'Right arm joint errors')
legend('shoulder_roll','shoulder_pitch','shoulder_yaw', 'shoulder_elbow')
grid on

ax4 = subplot(5,1,4); % left leg joint errors subplot
plot(time, left_hip_roll_error, time, left_hip_pitch_error, time, left_hip_yaw_error, time, left_knee_error, time, left_ankle_roll_error, time, left_ankle_pitch_error)
title(ax4,'Left foot joint errors')
legend('F_x','F_y','F_z', 'M_x', 'M_y', 'M_z')
grid on

ax5 = subplot(5,1,5); % right leg joint errors subplot
plot(time, right_hip_roll_error, time, right_hip_pitch_error, time, right_hip_yaw_error, time, right_knee_error, time, right_ankle_roll_error, time, right_ankle_pitch_error)
title(ax5,'Right foot joint errors')
legend('F_x','F_y','F_z', 'M_x', 'M_y', 'M_z')
grid on
