% INITCOORDINATOR initializes the robot configuration for running
%                'COORDINATOR' demo. 
%

%% --- Initialization ---

% Feet in contact (COORDINATOR DEMO ONLY)
CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT  = [1 1];

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
sat.torque = 500;

%% Control gains

% PARAMETERS FOR TWO FEET BALANCING
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 2)
    gain.PCOM                 = diag([50  50  50]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = 0*sqrt(gain.PCOM);

    gain.PAngularMomentum     = 10 ;
    gain.DAngularMomentum     = 2*sqrt(gain.PAngularMomentum);

    % Impedances acting in the null space of the desired contact forces 

    impTorso            = [10   10   20
                            0    0    0]; 
                        
    impArms             = [10   10    10    10  10 
                            0    0     0     0   0 ];
                        
    impLeftLeg          = [ 30   30   30    60   10  10
                             0    0    0     0    0   0]; 

    impRightLeg         = [ 30   30   30    60   10  10
                             0    0    0     0    0   0]; 
    
                         
    intTorso            = [0   0    0]; 
    
    intArms             = [0   0    0    0  0];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0     0  0    0  0];   
                                               
end

% PARAMETERS FOR ONE FOOT BALANCING
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 1)
    
    gain.PCOM                 = diag([50   100  50]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = diag([  0    0   0]);

    gain.PAngularMomentum     = 1 ;
    gain.DAngularMomentum     = 2*sqrt(gain.PAngularMomentum);

    % Impedances acting in the null space of the desired contact forces 
    
    intTorso            = [0   0    0];
    
    intArms             = [0   0    0    0  0];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0    0    0    0  0];  
    
    scalingImp          = 1.5;
    
    impTorso            = [20   20   30
                            0    0    0]*scalingImp;
                        
    impArms             = [15   15    15   10  10
                            0    0     0    0   0 ]*scalingImp;
                        
    impLeftLeg          = [ 30   30   30   120   10  10
                             0    0    0     0    0   0]*scalingImp; 

    impRightLeg         = [ 30   30   30    60   10  10
                             0    0    0     0    0   0]*scalingImp; 
                            
end
sat.integral              = 0;
gain.integral             = [intTorso,intArms,intArms,intLeftLeg,intRightLeg];
gain.impedances           = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
gain.dampings             = 0*sqrt(gain.impedances);
gain.increasingRatesImp   = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
sat.impedences            = [80   25    1400];

if (size(gain.impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end
% Smoothing time gain scheduling (YOGA DEMO ONLY)
gain.SmoothingTimeGainScheduling = 0.02;


%% References for CoM trajectory (COORDINATOR DEMO ONLY)

% that the robot waits before starting the left-and-right 
Config.noOscillationTime = 0;   

if Config.DEMO_MOVEMENTS && sum(Config.LEFT_RIGHT_FOOT_IN_CONTACT) == 2
        
    Config.directionOfOscillation = [0;1;0];
    % amplitude of oscillations in meters
    Config.amplitudeOfOscillation = 0.02;
    % frequency of oscillations in hertz
    Config.frequencyOfOscillation = 0.2;
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

% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;%1/3;  
torsionalFrictionCoefficient = 2/150;

%physical size of foot
feet_size                    = [ -0.16  0.16   ;   % xMin, xMax
                                 -0.075 0.075 ];   % yMin, yMax    
                             
gain.footSize                = [ -0.16  0.16   ;   % xMin, xMax
                                 -0.075 0.075 ];   % yMin, yMax    

fZmin                        = 10;



%% Regularization parameters
Reg.pinvDamp_nu_b = 1e-7;
Reg.pinvDamp      = 0.01; 
Reg.pinvTol       = 1e-5;
Reg.impedances    = 0.1;
Reg.dampings      = 0;
Reg.HessianQP     = 1e-5;
