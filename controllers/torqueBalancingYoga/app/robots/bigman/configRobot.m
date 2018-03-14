% CONFIGROBOT initializes parameters specific of a particular robot
%             (e.g., Bigman)
%

%% --- Initialization ---
Config.ON_GAZEBO         = true;
ROBOT_DOF              = 25;
ROBOT_DOF_FOR_SIMULINK  = eye(ROBOT_DOF);
WBT_robotName = 'bigman';
WBT_wbiList = '(WaistSag,WaistLat,WaistYaw,LShSag,LShLat,LShYaw,LElbj,LForearmPlate,RShSag,RShLat,RShYaw,RElbj,RForearmPlate,LHipSag,LHipLat,LHipYaw,LKneeSag,LAnkSag,LAnkLat,RHipSag,RHipLat,RHipYaw,RKneeSag,RAnkSag,RAnkLat)';

tSwitch            = [10 20 30 40 50 60 70 80 90 100];
tEnd               = tSwitch(end) + 10;

% Frames list
Frames.BASE              = 'Waist'; 
Frames.IMU               = 'imu_link';
Frames.LEFT_FOOT         = 'l_sole';
Frames.RIGHT_FOOT        = 'r_sole';
Frames.COM               = 'com';

Config.ON_WALKMAN_PILOT_PC = false;
Config.ON_REAL_WALKMAN     = false;

% Simulation time in seconds
Config.SIMULATION_TIME = inf;

% Available movesets: 1 = air_1; 2 = air_2, 3 = yoga
Config.moveset = 3;

if Config.ON_WALKMAN_PILOT_PC

    setenv('ROBOTOLOGY_SUPERBUILD_ROOT','/home/ahmad/robotology-superbuild');
    current_path = getenv('PATH');
    setenv('PATH',fullfile(current_path, ':/home/ahmad/robotology-superbuild/build/install/bin'));
    current_ld_library_path = getenv('LD_LIBRARY_PATH');
    setenv('LD_LIBRARY_PATH',fullfile(current_ld_library_path, ':/home/ahmad/robotology-superbuild/build/install/lib'));
    setenv('YARP_DATA_DIRS','/home/ahmad/robotology-superbuild/build/install/share/yarp:/home/ahmad/robotology-superbuild/build/install/share/iCub:/home/ahmad/robotology-superbuild/build/install/share/codyco');
end

Config.smoothingTimeTranDynamics   = 0.05;

% calibration delta for legs joints
newOffsets = [0.12753
              2.554397
              2.024109
             -2.243889
              1.644721
             -0.39407
             -0.642021
             -0.183969
              0.233145
             -1.176114
              0.866804
              0.247708]; % [deg]

% calibration delta (real - desired) 
calibDelta = newOffsets*pi/180;
Ports.IMU_CALIB = '/bigman/imu_link/inertial';

if Config.ON_REAL_WALKMAN == 0
    % ONLY FOR SIMULATION
    calibDelta = 0.*calibDelta;
    Ports.IMU_CALIB = '/bigman/inertial';
end

if Config.moveset == 3
   
    tSwitch  = [10,20,30,40];
    tEnd     = tSwitch(end) + 10;    
end

% Config.USE_MOTOR_REFLECTED_INERTIA: if set to true, motors reflected
% inertias are included in the system mass matrix.
Config.USE_MOTOR_REFLECTED_INERTIA = false;

% Config.USE_IMU4EST_BASE: if set to false, the base frame is estimated by 
% assuming that either the left or the right foot stay stuck on the ground. 
% Which foot the controller uses depends on the contact forces acting on it. 
% If set to true, the base orientation is estimated by using the IMU, while
% the base position by assuming that the origin of either the right or the
% left foot do not move. 
Config.USE_IMU4EST_BASE  = false;

% Config.YAW_IMU_FILTER and Config.PITCH_IMU_FILTER: when the flag
% Config.USE_IMU4EST_BASE = true, then the orientation of the floating base is
% estimated as explained above. However, the foot is usually perpendicular
% to gravity when the robot stands on flat surfaces, and rotation about the
% gravity axis may be de to the IMU drift in estimating this angle. Hence,
% when either of the flags Config.YAW_IMU_FILTER or Config.PITCH_IMU_FILTER
% is set to true, then the yaw and/or pitch angles of the contact foot are
% ignored and kept equal to the initial values.
Config.FILTER_IMU_YAW    = true;
Config.FILTER_IMU_PITCH  = true;

% Config.CORRECT_NECK_IMU: when set equal to true, the kineamtics from the
% IMU and the contact foot is corrected by using the neck angles. If it set
% equal to false, recall that the neck is assumed to be in (0,0,0).
Config.CORRECT_NECK_IMU  = true;

% Config.USE_QP_SOLVER: if set to true, a QP solver is used to account for 
% inequality constraints of contact wrenches.
Config.USE_QP_SOLVER     = true; 

% Ports name list
Ports.IMU              = '/bigman/inertial';
Ports.NECK_POS          = ['/' WBT_robotName '/head/state:o'];
Ports.WBD_LEFTLEG_EE  = '/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o';
Ports.WBD_RIGHTLEG_EE = '/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o';

