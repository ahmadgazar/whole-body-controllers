% INITCOORDINATOR initializes the robot configuration for running
%                'COORDINATOR' demo. 
%

%% --- Initialization ---

% Feet in contact (COORDINATOR DEMO ONLY)
Config.LEFT_RIGHT_FOOT_IN_CONTACT = [1 0];


%initial conditions of xi_dot
if Config.LEFT_RIGHT_FOOT_IN_CONTACT(1) == 1 && Config.LEFT_RIGHT_FOOT_IN_CONTACT(2) == 0
  Config.xi_dot_initial = [0 0 log(324) 0 0 0 0 0 0 0 0 0];
  
elseif Config.LEFT_RIGHT_FOOT_IN_CONTACT(1) == 0 && Config.LEFT_RIGHT_FOOT_IN_CONTACT(2) == 1
    Config.xi_dot_initial = [0 0 0 0 0 0 0 0 log(300) 0 0 0];
else
    Config.xi_dot_initial = [0 0 log(150) 0 0 0 0 0 log(150) 0 0 0];
end

% Initial foot on ground. If false, right foot is used as default contact
% frame (this does not means that the other foot cannot be in contact too).
% (COORDINATOR DEMO ONLY)
Config.LEFT_FOOT_IN_CONTACT_AT_0 = true;

% If true, the robot CoM will follow a desired reference trajectory (COORDINATOR DEMO ONLY)
Config.DEMO_MOVEMENTS = false;

% If equal to true, the desired streamed values of the center of mass 
% are smoothed internally 
Config.SMOOTH_COM_DES = false;   

% If equal to true, the desired streamed values of the postural tasks are
% smoothed internally 
Config.SMOOTH_JOINT_DES = false;   

% torque saturation
Sat.torque = 34;
Sat.xiDot  = 10;

%% Control gains
Sm.CoM_desired_z = [0.5553; 0.5708; 0.5857; 0.5909; 0.6039; 0.58; 0.6267; 0.64];                     

% PARAMETERS FOR TWO FEET BALANCING
if sum(Config.LEFT_RIGHT_FOOT_IN_CONTACT) == 2
    
    Gain.KP_COM               = diag([50  150  50])/2;     % Kp(x_dot_CoMDesired -x_dotCoM), increasing this too much is not good since x_dotCoM is computed as x_dotCoM = Jc*nu, where nu is not accurately estimated 
    Gain.KD_COM               = 2*sqrt(Gain.KP_COM)*0;      % Kd(x_ddot_CoMDesired - x_ddot_CoM), start with zero first
    Gain.KI_COM               = diag([30   100    30])/4;  % Ki(x_CoMDesired - x_CoM)
    Gain.KP_AngularMomentum   = diag([200   150    150])/10;
    Gain.KD_AngularMomentum   = 2*sqrt(Gain.KP_AngularMomentum)*0;
    Gain.KI_AngularMomentum   = 10;
    Gain.k_xi                 = 0;
    Gain.k_t                  = diag([10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10]);
    
   % Impedances acting in the null space of the desired contact forces    
    impTorso            = [30   30   30];
    
    impArms             = [15   15   15    8];
                        
    impLeftLeg          = [60   5   60    60   50   5];

    impRightLeg         = [60   5   60    60   50   5];                                            
end

% PARAMETERS FOR ONE FOOT BALANCING
if sum(Config.LEFT_RIGHT_FOOT_IN_CONTACT) == 1
    
%     Gain.KP_COM               = diag([50  50  50])/2;     % Kp(x_dot_CoMDesired -x_dotCoM), increasing this too much is not good since x_dotCoM is computed as x_dotCoM = Jc*nu, where nu is not accurately estimated 
%     Gain.KD_COM               = diag([5   5    10]);      % Kd(x_ddot_CoMDesired - x_ddot_CoM), start with zero first
%     Gain.KI_COM               = diag([30   30    30])/5;  % Ki(x_CoMDesired - x_CoM)
%     Gain.KP_AngularMomentum   = diag([200   150    150])/10;
%     Gain.KD_AngularMomentum   = diag([1   1    1]); %2*sqrt(Gain.KP_AngularMomentum)*0;
%     Gain.KI_AngularMomentum   = 10;
%     Gain.k_xi                 = 0;
%     Gain.k_t                  = diag([5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5]);
    
    Gain.KP_COM               = diag([30  30  30]);   % Kp(x_dot_CoMDesired -x_dotCoM), increasing this too much is not good since x_dotCoM is computed as x_dotCoM = Jc*nu, where nu is not accurately estimated 
    Gain.KD_COM               = diag([15  15  15]);   % Kd(x_ddot_CoMDesired - x_ddot_CoM), start with zero first
    Gain.KI_COM               = diag([10  10  10]);   % Ki(x_CoMDesired - x_CoM)
    Gain.KP_AngularMomentum   = diag([30   30    30]);
    Gain.KD_AngularMomentum   = diag([15   15    15]);
    Gain.KI_AngularMomentum   = 100;
    Gain.k_xi                 = 0;
    Gain.k_t = diag([5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5]);


    % Impedances acting in the null space of the desired contact forces    
    impTorso            = [30   30   30];
    
    impArms             = [15   15   15    8];
                        
    impLeftLeg          = [60   60   60    60   20   20];

    impRightLeg         = [30   30   30    60    10   10];   
end

Gain.impedances         = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
Gain.dampings           = 0*sqrt(Gain.impedances);

if (size(Gain.impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end

% Smoothing time gain scheduling (YOGA DEMO ONLY)
Gain.SmoothingTimeGainScheduling = 0.02;

%% Parameters for motors reflected inertia

% transmission ratio
Config.Gamma = 0.01*eye(ROBOT_DOF);

% motors inertia (Kg*m^2)
legsMotors_I_m           = 0.0827*1e-4;
torsoPitchRollMotors_I_m = 0.0827*1e-4;
torsoYawMotors_I_m       = 0.0585*1e-4;
armsMotors_I_m           = 0.0585*1e-4;
Config.I_m               = diag([torsoPitchRollMotors_I_m*ones(2,1);
                                 torsoYawMotors_I_m;
                                 armsMotors_I_m*ones(8,1);
                                 legsMotors_I_m*ones(12,1)]);

% parameters for coupling matrices                            
t  = 0.625;
r  = 0.022;
R  = 0.04;

% coupling matrices
T_LShoulder = [-1  0  0;
               -1 -t  0;
                0  t -t];

T_RShoulder = [ 1  0  0;
                1  t  0;
                0 -t  t];

T_torso = [0   -0.5     0.5;
           0    0.5     0.5;
           r/R  r/(2*R) r/(2*R)];
       
if Config.INCLUDE_COUPLING
       
    Config.T = blkdiag(T_torso,T_LShoulder,1,T_RShoulder,1,eye(12));
else          
    Config.T = eye(ROBOT_DOF);
end

% gain for feedforward term in joint torques calculation. Valid range: a
% value between 0 and 1
Config.K_ff  = 0;

%% References for CoM trajectory (COORDINATOR DEMO ONLY)

% that the robot waits before starting the left-and-right 
Config.noOscillationTime = 1;   

if Config.DEMO_MOVEMENTS && sum(Config.LEFT_RIGHT_FOOT_IN_CONTACT) == 2
        
    Config.directionOfOscillation = [0;1;0];
    % amplitude of oscillations in meters
    Config.amplitudeOfOscillation = 0.02;
    % frequency of oscillations in hertz
    Config.frequencyOfOscillation = 0.5;
else
    Config.directionOfOscillation  = [0;0;0];
    Config.amplitudeOfOscillation  = 0.0;  
    Config.frequencyOfOscillation  = 0.0;
end

%% State machine parameters

% smoothing time for joints and CoM
Sm.smoothingTimeCoM_Joints = 1; 

% time between two yoga positions (YOGA DEMO ONLY)
Sm.joints_pauseBetweenYogaMoves = 0;

% contact forces threshold (YOGA DEMO ONLY)
Sm.wrench_thresholdContactOn  = 1;
Sm.wrench_thresholdContactOff = 1;

% threshold on CoM and joints error (YOGA DEMO ONLY)
Sm.CoM_threshold                = 0; 
Sm.joints_thresholdNotInContact = 0;
Sm.joints_thresholdInContact    = 0;

% initial state for state machine (YOGA DEMO ONLY)
Sm.stateAt0 = 1;

% delta to be summed to the reference CoM position (YOGA DEMO ONLY)
Sm.CoM_delta = [0; 0; 0];

% joint references (YOGA DEMO ONLY)
Sm.joints_references   = zeros(1,ROBOT_DOF);
Sm.joints_leftYogaRef  = zeros(1,ROBOT_DOF+1);
Sm.joints_rightYogaRef = zeros(1,ROBOT_DOF+1);

% configuration parameters for state machine (YOGA DEMO ONLY) 
Sm.tBalancing               = 1;
Sm.tBalancingBeforeYoga     = 1;
Sm.skipYoga                 = false;
Sm.demoOnlyBalancing        = false;
Sm.demoStartsOnRightSupport = false;
Sm.yogaAlsoOnRightFoot      = false;
Sm.yogaInLoop               = false;

%% Constraints for QP for balancing

% The friction cone is approximated by using linear interpolation of the circle. 
% So, numberOfPoints defines the number of points used to interpolate the circle 
% in each cicle's quadrant
numberOfPoints               = 4;  
delta_c                      = 1/3; %friction coefficient
forceFrictionCoefficient     = 1/3;
delta_x                      = 0.07*2;    %CoP along x must be inside the support polygon i.e foot size along X
delta_y                      = 0.03*2;    %CoP along y must be inside the support polygon i.e foot size along Y
delta_z                      = 1/75;    % torsional coefficient 
torsionalFrictionCoefficient = 1/75; 

% physical size of the foot                             
feet_size                    = [-0.07  0.07;   % xMin, xMax
                                -0.03  0.03];  % yMin, yMax    
 
fZmin                        = 10;

%% Regularization parameters
Reg.pinvDamp_nu_b = 1e-7;
Reg.pinvDamp      = 1; 
Reg.pinvTol       = 1e-5;
Reg.impedances    = 0.1;
Reg.dampings      = 0;
Reg.HessianQP     = 1e-5;
