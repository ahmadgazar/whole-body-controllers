% INITSTATEMACHINEYOGA initializes the robot configuration for running
%                      'YOGA' demo. 
% 

%% Initialization
Config.LEFT_RIGHT_FOOT_IN_CONTACT = [1 1];

% Initial foot on ground. If false, right foot is used as default contact
% frame (this does not means that the other foot cannot be in contact too).
% (COORDINATOR DEMO ONLY)
Config.LEFT_FOOT_IN_CONTACT_AT_0 = true;

% If true, the robot CoM will follow a desired reference trajectory (COORDINATOR DEMO ONLY)
Config.DEMO_MOVEMENTS = false;
    
Config.SMOOTH_COM_DES = true;   % If equal to one, the desired streamed values 
                                       % of the center of mass are smoothed internally 
Config.SMOOTH_JOINT_DES  = true;    % If equal to one, the desired streamed values 
                                       % of the postural tasks are smoothed internally 
%% Regularization parameters

    Reg.pinvDamp_nu_b = 1e-7;
    Reg.pinvDamp      = 1; 
    Reg.pinvTol       = 1e-5;
    Reg.impedances    = 0.1;
    Reg.dampings      = 0;
    Reg.HessianQP     = 1e-7;    
    Sat.torque                 = 5000;
    
    %Smoothing time for time varying impedances
    Gain.SmoothingTimeGainScheduling  = 2;  

    %Smoothing time for time-varying constraints
    Config.smoothingTimeTranDynamics  = 0.02;

 %% COM AND JOINT GAINS 
    Gain.KP_COM      =    [15    20   15   % state ==  1   TWO FEET BALANCING
                            25    35   25   % state ==  2   COM TRANSITION TO LEFT 
                            25    35   25   % state ==  3   LEFT FOOT BALANCING
                            15    20   15   % state ==  4   YOGA LEFT FOOT 
                            15    20   15   % state ==  5   PREPARING FOR SWITCHING 
                            15    20   15   % state ==  6   LOOKING FOR CONTACT
                            15    20   15   % state ==  7   TRANSITION TO INITIAL POSITION 
                            15    20   15   % state ==  8   COM TRANSITION TO RIGHT FOOT
                            15    20   15   % state ==  9   RIGHT FOOT BALANCING
                            15    20   15   % state ==  10  YOGA RIGHT FOOT 
                            15    20   15   % state ==  11  PREPARING FOR SWITCHING 
                            15    20   15   % state ==  12  LOOKING FOR CONTACT
                            15    20   15]; % state ==  13  TRANSITION TO INITIAL POSITION

    Gain.KD_COM  = 2*sqrt(Gain.KP_COM); % which is the integral term from the point of view of the linear momentum    
    Gain.KP_AngularMomentum  = 0.95;
    Gain.KD_AngularMomentum  = 2*sqrt(Gain.KP_AngularMomentum); %This notation should be changed to be KI_AngularMomentum


                       %   TORSO  %%        LEFT ARM   %%            RIGHT ARM   %%             LEFT LEG            %%        RIGHT LEG           %% 
    Gain.impedances  = [10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  1  TWO FEET BALANCING
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  2  COM TRANSITION TO LEFT 
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  3  LEFT FOOT BALANCING
                        80  120   80,  40   40    40   40   40,  40   40    40   40   40,  80   80  250   200   50  50,  70   70   70    70   50  50   % state ==  4  YOGA LEFT FOOT 
                        30   30   30,  10   10    10    8   10,  10   10    10    8   10,  30   50  300    60   50  50,  30   50   30    60   50  50   % state ==  5  PREPARING FOR SWITCHING 
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  6  LOOKING FOR CONTACT
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  7  TRANSITION TO INITIAL POSITION 
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  8  COM TRANSITION TO RIGHT FOOT
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state ==  9  RIGHT FOOT BALANCING
                        80  120   80,  40   40    40   40   40,  40   40    40   40   40,  70   70   70    70   50  50,  80   80  250   200   50  50   % state == 10  YOGA RIGHT FOOT 
                        30   30   30,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50  300    60   50  50   % state == 11  PREPARING FOR SWITCHING 
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50   % state == 12  LOOKING FOR CONTACT
                        10   30   20,  10   10    10    8   10,  10   10    10    8   10,  30   50   30    60   50  50,  30   50   30    60   50  50]; % state == 13  TRANSITION TO INITIAL POSITION

    Gain.impedances(1:3,:) = Gain.impedances(1:3,:)/5;
    Gain.impedances(7:9,:) = Gain.impedances(7:9,:)/5; 
    Gain.dampings    = 0*sqrt(Gain.impedances(1,:));  

% Smoothing time gain scheduling (YOGA DEMO ONLY)
Gain.SmoothingTimeGainScheduling = 2;

%% FINITE STATE MACHINE SPECIFIC PARAMETERS
% configuration parameters for state machine (YOGA DEMO ONLY) 
Sm.tBalancing               = 1;
Sm.tBalancingBeforeYoga     = 1;

Sm.demoOnlyBalancing        = false;
Sm.skipYoga                 = false;
Sm.demoStartsOnRightSupport = false;
Sm.yogaAlsoOnRightFoot      = true;
Sm.yogaInLoop               = false;

% contact forces threshold
Sm.CoM.threshold                 = 0.01;
Sm.wrench_thresholdContactOn     = 5;      % Force threshole above which contact is considered stable
Sm.wrench_thresholdContactOff    = 450;    % Force threshole under which contact is considered off

% threshold on CoM and joints error )
Sm.CoM_threshold                = 0.01; 
Sm.joints_thresholdNotInContact = 5;
Sm.joints_thresholdInContact    = 50;


% time between two yoga positions 
Sm.joints_pauseBetweenYogaMoves = 3;
Sm.tBalancingOneFoot             = 1;
Sm.stateAt0                      = 1;
Sm.DT                            = 1;
Sm.waitingTimeAfterYoga          = 0;

% smoothing time for joints and CoM
Sm.smoothingTimeCoM_Joints          = [5;   %% state ==  1  TWO FEET BALANCING
                                    3;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                                    3;   %% state ==  3  LEFT FOOT BALANCING 
                                    4;   %% state ==  4  YOGA LEFT FOOT
                                    5;   %% state ==  5  PREPARING FOR SWITCHING
                                    2;   %% state ==  6  LOOKING FOR CONTACT 
                                    4;   %% state ==  7  TRANSITION INIT POSITION
                                    5;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                                    5;   %% state ==  9  RIGHT FOOT BALANCING 
                                    4;   %% state == 10  YOGA RIGHT FOOT
                                    5;   %% state == 11  PREPARING FOR SWITCHING
                                    1;   %% state == 12  LOOKING FOR CONTACT 
                                    5];  %% state == 13  TRANSITION INIT POSITION

Sm.CoM_delta      =  [0.00,    0.00,    0.00;       %% state ==  1  TWO FEET BALANCING NOT USED
                      0.00,    0.015,   0.00;       %% state ==  2  COM TRANSITION TO LEFT FOOT: THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
                      0.00,    0.00,    0.050;       %% state ==  3  LEFT FOOT BALANCING 
                      0.00,    0.025,   0.250;       %% state ==  4  YOGA LEFT FOOT
                      0.00,    0.00,   0.00;       %% state ==  5  PREPARING FOR SWITCHING
                      0.0,   -0.07,  -0.015;      %% state ==  6  LOOKING FOR CONTACT 
                      0.01,  -0.02,  -0.025;      %% state ==  7  TRANSITION INIT POSITION
                      0.0,     0.00,   0.00;       %% state ==  8  COM TRANSITION TO RIGHT FOOT
                      0.00,    0.00,   0.050;       %% state ==  9  RIGHT FOOT BALANCING 
                      0.00,    0.00,    0.250;       %% state == 10  YOGA RIGHT FOOT
                      0.00,    0.00,    0.00;       %% state == 11  PREPARING FOR SWITCHING
                      0.06,    0.025,  0.05;      %% state == 12  LOOKING FOR CONTACT 
                      0.00,    0.00   0.00];     %% state == 13  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
                  
%Sm.tBalancing      =  0;

Sm.joints_references = [zeros(1,ROBOT_DOF);     %
                    [-0.0350,-0.2000, 0.0400, ...                          %% state == 2  COM TRANSITION TO LEFT 
                     -0.1875, 0.1000, 0.2500, 0.8700, 0.0000 ...           %
                     -0.1100, 1.4000, 0.0125, 0.8700, 0.0000 ...           %
                     -0.0025,-0.1100,-0.0000, 0.0000, 0.0150, 0.1500, ...  %  
                      0.0000, 0.1000,-0.0025,-0.0050, 0.0075,-0.1150];     %  
                    [ 0.0875,-0.2000, 0.0150, ...                          %% state == 3  LEFT FOOT BALANCING
                      0.0750, 0.1000, 0.3050, 0.7925, 0.0000 ...           %    
                      0.0775, 1.4000, 0.0150, 0.6200, 0.0000 ...           %
                     -0.0015,-0.1100,-0.0000, 0.0000, 0.0150, 0.1650, ...  %  
                      0.0000, 0.0800,-0.0025,-0.0050, 0.0075,-0.1150];     % 
                    [ 0.0875, 0.0275, 0.0150, ...                          %% state == 4  YOGA LEFT FOOT, THIS REFERENCE IS IGNORED 
                      0.1250, 0.8000, 0.3050, 0.7925, 0.0000 ...           %
                      0.0550, 0.7000, 0.0250, 0.6225, 0.0000 ...           %
                      0.0525,-0.2575, 0.0025,-0.2100,-0.0950, 0.1950, ...  %
                      0.0125, 0.4375, 0.0100,-0.1575,-0.0725,-0.2925];     %
                    [-0.0350, 0.0775, 0.0425, ...                          %% state == 5  PREPARING FOR SWITCHING
                     -0.1500, 0.8575, 0.2425, 0.8700, 0.0000 ...           %
                     -0.1500, 0.8575, 0.2425, 0.8700, 0.0000 ...           %
                     -0.0025,-0.1900,-0.0000,-0.0025, 0.0150, 0.1625, ...  %  
                      0.0005, 0.1925,-0.0014,-0.0031  0.0075 -0.1150];     %                                  %
                    [-0.0875, 0.0250, 0.0150, ...                          %% state == 6  LOOKING FOR CONTACT
                      0.1250, 0.9000, 0.3050, 0.7950, 0.0000 ...           %
                      0.0575, 0.9000, 0.3350, 0.6225, 0.0000 ...           %
                      0.0100,-0.0750,-0.0000,-0.0325, 0.0250, 0.1375, ...  %
                     -0.0025, 0.0300, 0.0100,-0.0000, 0.0025,-0.0275];     %   
                      zeros(1,ROBOT_DOF);                                  %% state == 7  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
                    [ 0.0875, 0.2000, 0.0150, ...                          %% state == 8  COM TRANSITION TO RIGHT FOOT
                      0.1250, 1.4150, 0.3050, 0.7925, 0.0000 ...           %
                      0.0575, 0.2575, 0.3350, 0.6225, 0.0000 ...           %
                      0.0100,-0.0750,-0.0000,-0.0120, 0.0250, 0.1375, ...  %
                     -0.0025, 0.0300, 0.0100,-0.0000, 0.0025,-0.0275];     % 
                    [ 0.0864, 0.2000, 0.0150, ...                          %% state == 9  RIGHT FOOT BALANCING
                      0.1250, 0.8125, 0.3050, 0.7925, 0.0000 ...           %    
                      0.0575, 0.0800, 0.3350, 0.6225, 0.0000 ...           %
                      0.0005, 0.0800,-0.0025,-0.0050, 0.0075,-0.1150, ...  %  
                     -0.0025,-0.1100,-0.0000, 0.0000, 0.0150, 0.1650];     %  
                      zeros(1,ROBOT_DOF);                                  %% state == 10  YOGA RIGHT FOOT, THIS REFERENCE IS IGNORED  
                    [-0.0350, 0.0775, 0.0450, ...                          %% state == 11  PREPARING FOR SWITCHING
                     -0.1500, 0.8600, 0.2425, 0.8700, 0.0000 ...           %
                     -0.1500, 0.8600, 0.2425, 0.8700, 0.0000 ...           %
                      0.0005, 0.0800,-0.0025,-0.0050, 0.0075,-0.1150, ...  %  
                     -0.0025,-0.1100,-0.0000, 0.0005, 0.0150, 0.1625];     %                                  %
                    [ 0.0875, 0.0250, 0.0150, ...                          %% state == 12  LOOKING FOR CONTACT
                      0.1250, 0.8125, 0.3050, 0.7925, 0.0000 ...           %
                      0.0550, 0.6800, 0.3350, 0.6225, 0.0000 ...           %
                     -0.0025, 0.0225, 0.0100,-0.0025, 0.0025,-0.0275, ...  %
                      0.0105,-0.0750,-0.0000,-0.0125, 0.0250, 0.1375];     %   
                      zeros(1,ROBOT_DOF)];                                 %% state == 13  BALANCING TWO FEET, THIS REFERENCE IS IGNORED                     

 
q1 =        [-0.0800,-0.1000, 0.4500, ...
             -1.2300, 0.7000, 1.2000, 0.9950, 0.0000, ... 
             -1.0700, 1.2000, 0.5500, 1.0950, 0.0000, ...
              0.2100, 0.2950, 0.0005,-0.1750,-0.1045, 0.0700, ...
              0.3485, 0.4000,-0.0005,-0.3650,-0.0550,-0.0875];

q2 =        [-0.0800,-0.1000, 0.4500, ...
             -1.2300, 0.7000, 1.2000, 0.9950, 0.0000, ...
             -1.0700, 1.2000, 0.5500, 1.0950, 0.0000, ...
              0.2090, 0.2950, 0.0005,-0.1750,-0.1045, 0.0700, ... 
              0.3715, 0.9600, 1.3250,-1.6595, 0.6375,-0.0615];
          
q3 =        [-0.0850,-0.4275, 0.0800,...
              0.1400, 1.3575, 0.2475, 0.3050, 0.0000, ...
             -0.3180, 1.3575, 0.7375, 0.3050, 0.0000, ...
              0.2090, 0.2950, 0.0005,-0.1750,-0.1045, 0.0700, ...
              0.3715, 0.9600, 1.3255,-1.6595, 0.6375,-0.0615];
          
q4 =        [-0.0850,-0.4275, 0.0800,...
              0.1400, 1.3585, 0.2475, 0.3050, 0.0000, ...
             -0.3150, 1.3585, 0.7375, 0.3050, 0.0000, ...
              0.2100, 0.3475, 0.0005,-0.1750,-0.1045, 0.0700,...
              0.3515, 1.3100, 1.3255,-0.0200, 0.6375,-0.0615];
          
q5 =        [-0.0800,-0.1275, 0.4500, ...
             -1.1625, 0.6675, 0.4965, 0.9945, 0.0000, ...
             -1.0725, 1.2900, 0.2500, 1.0950, 0.0000, ...
              0.2090, 0.3475, 0.0005,-0.1750,-0.1050, 0.0700,...
              0.3525, 1.3100, 1.3250,-0.0200, 0.6370,-0.0600];
          
q6 =        [-0.0850,-0.4275, 0.0800,...
              0.1400, 1.3585, 0.2475, 0.3050, 0.0000, ...
             -0.4200, 1.4585, 0.7375, 0.3025, 0.0000, ...
              0.2100, 0.3475, 0.0005,-0.1750,-0.1075, 0.0700,...
              0.3525, 1.3100, 1.3250,-0.0200, 0.6375,-0.0600];
          
q7 =        [-0.0850,-0.4275, 0.0800,...
              0.1400, 1.3585, 0.2475, 0.3050, 0.0000, ...
             -0.4200, 1.4585, 0.7375, 0.3025, 0.0000, ...
              0.2100, 0.3475, 0.0005,-0.1750,-0.1075, 0.0700,...
              0.3525, 1.3100, 1.3250,-1.6200, 0.6375,-0.0600];
          
q8 =        [-0.0850,-0.4275, 0.0800,...
              0.1400, 1.3585, 0.2475, 0.3050, 0.0000, ...
             -0.4200, 1.4585, 0.7375, 0.3025, 0.0000, ...
              0.2100, 0.3475, 0.0005,-0.1750,-0.1075, 0.0700,...
              0.3525, 1.3100, 1.3250,-0.0200, 0.6375,-0.0600];

%% Symmetry for right foot Yoga          
Sm.joints_leftYogaRef = [0,                            q1;
                     1*Sm.smoothingTimeCoM_Joints(4),q2;
                     2*Sm.smoothingTimeCoM_Joints(4),q3;
                     3*Sm.smoothingTimeCoM_Joints(4),q4;
                     4*Sm.smoothingTimeCoM_Joints(4),q5;
                     5*Sm.smoothingTimeCoM_Joints(4),q6;
                     6*Sm.smoothingTimeCoM_Joints(4),q7;
                     7*Sm.smoothingTimeCoM_Joints(4),q8];
                 
Sm.joints_rightYogaRef  = Sm.joints_leftYogaRef;
Sm.joints_rightYogaRef(:,1) = [0;
                               1*Sm.smoothingTimeCoM_Joints(10);
                               2*Sm.smoothingTimeCoM_Joints(10);
                               3*Sm.smoothingTimeCoM_Joints(10);
                               4*Sm.smoothingTimeCoM_Joints(10);
                               5*Sm.smoothingTimeCoM_Joints(10);
                               6*Sm.smoothingTimeCoM_Joints(10);
                               7*Sm.smoothingTimeCoM_Joints(10)];
                           
% MIRROR YOGA LEFT MOVESET FOR RIGHT YOGA					 				 
for i = 1:size(Sm.joints_rightYogaRef,1)				
	Sm.joints_rightYogaRef(i,2:4)           = [Sm.joints_rightYogaRef(i,2) -Sm.joints_rightYogaRef(i,3) -Sm.joints_rightYogaRef(i,4)];
	rightArm                                =  Sm.joints_rightYogaRef(i,end-16:end-12);
	Sm.joints_rightYogaRef(i,end-16:end-12) =  Sm.joints_rightYogaRef(i,end-21:end-17);
	Sm.joints_rightYogaRef(i,end-21:end-17) =  rightArm;
	rightLeg                                =  Sm.joints_rightYogaRef(i,end-5:end);
	Sm.joints_rightYogaRef(i,end-5:end)     =  Sm.joints_rightYogaRef(i,end-11:end-6);
	Sm.joints_rightYogaRef(i,end-11:end-6)  =  rightLeg;
end	 

%% References for CoM trajectory (COORDINATOR DEMO ONLY)

% that the robot waits before starting the left-and-right 
Config.noOscillationTime       = 0;   
Config.directionOfOscillation  = [0;0;0];
Config.amplitudeOfOscillation  = 0.0;  
Config.frequencyOfOscillation  = 0.0;

%% Parameters for motors reflected inertia

% inverse of the transmission ratio
Config.invGamma = 100*eye(ROBOT_DOF);
% torso yaw has a bigger reduction ratio
Config.invGamma(3,3) = 200;

% motors inertia (Kg*m^2)
legsMotors_I_m           = 0.0827*1e-4;
torsoPitchRollMotors_I_m = 0.0827*1e-4;
torsoYawMotors_I_m       = 0.0585*1e-4;
armsMotors_I_m           = 0.0585*1e-4;
Config.I_m               = diag([torsoPitchRollMotors_I_m*ones(2,1);
                                 torsoYawMotors_I_m;
                                 armsMotors_I_m*ones(8,1);
                                 legsMotors_I_m*ones(12,1)]);

% gain for feedforward term in joint torques calculation. Valid range: a
% value between 0 and 1
Config.K_ff     = 0;
%% Remapping of the references in order to fit the Bigman configuration and the iCub configuration
numOfStates     = size(Sm.joints_references,1);
ndof            = size(Sm.joints_references,2);
numOfStatesYoga = 8;

for kk = 1:numOfStates
    
    Sm.joints_references(kk,:) = from_iCub_To_Bigman_JointRemapper(Sm.joints_references(kk,:),ndof);
end

for kk = 1:numOfStatesYoga
    
    qjRemapped                    = from_iCub_To_Bigman_JointRemapper(Sm.joints_rightYogaRef(kk,2:end),ndof);
    Sm.joints_rightYogaRef(kk,2:end)   = qjRemapped;
    qjRemapped                    = from_iCub_To_Bigman_JointRemapper(Sm.joints_leftYogaRef(kk,2:end),ndof);
    Sm.joints_leftYogaRef(kk,2:end)   = qjRemapped;
end

q5 = from_iCub_To_Bigman_JointRemapper(q5,ndof);
q6 = from_iCub_To_Bigman_JointRemapper(q6,ndof);
q7 = from_iCub_To_Bigman_JointRemapper(q7,ndof);
q8 = from_iCub_To_Bigman_JointRemapper(q8,ndof);
%% References for CoM trajectory (COORDINATOR DEMO ONLY)

% that the robot waits before starting the left-and-right 
Config.noOscillationTime       = 0;   
Config.directionOfOscillation  = [0;0;0];
Config.amplitudeOfOscillation  = 0.0;  
Config.frequencyOfOscillation  = 0.0;

%% Parameters for motors reflected inertia

% inverse of the transmission ratio
Config.invGamma = 100*eye(ROBOT_DOF);
% torso yaw has a bigger reduction ratio
Config.invGamma(3,3) = 200;

% motors inertia (Kg*m^2)
legsMotors_I_m           = 0.0827*1e-4;
torsoPitchRollMotors_I_m = 0.0827*1e-4;
torsoYawMotors_I_m       = 0.0585*1e-4;
armsMotors_I_m           = 0.0585*1e-4;
whateverMotors           = 0.07*1e-4;
Config.I_m               = diag([torsoPitchRollMotors_I_m*ones(2,1);
                                 torsoYawMotors_I_m;
                                 armsMotors_I_m*ones(8,1);
                                 legsMotors_I_m*ones(12,1);
                                 whateverMotors*ones(2,1)]);

% gain for feedforward term in joint torques calculation. Valid range: a
% value between 0 and 1
Config.K_ff     = 0;

%% Constraints for QP for balancing

% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;%1/3;  
torsionalFrictionCoefficient = 2/150;

%physical size of foot
feet_size                = [ -0.16  0.16   ;   % xMin, xMax
                            -0.075 0.075 ];   % yMin, yMax    
                           

fZmin                        = 10;

%% Cleanup
clear q1 q2 q3 q4;
