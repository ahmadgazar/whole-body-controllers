function [xi_dot, errorCoM, errorCoM_dot, errorCoM_ddot, errorCoM_dot_angular, errorCoM_ddot_angular, error_H_linear,error_H_angular, HDot_linear_ft,  HDot_angular_ft] = Momentum_acceleration_Controller(M, nu, w_H_l_sole, w_H_r_sole, Left_Right_F_T_Sensors, ...
                                                                 J_CoM, desired_x_dx_ddx_CoM,  gainsPCOM, ...
                                                                 gainsDCOM, gainsICOM, H, constraints, intHw, xCoM, state, tau, Sigma, F, Gain, Reg, xi)
% Mass of the robot
m              = M(1,1);

% Gravity acceleration
gravAcc        = 9.81;
gravityWrench  = [zeros(2,1);
                  -m*gravAcc;
                  zeros(3,1)];

pos_leftFoot   = w_H_l_sole(1:3,4);
pos_rightFoot  = w_H_r_sole(1:3,4);

% Application point of the contact force on the -right foot w.r.t. CoM
Pr             = pos_rightFoot - xCoM; 
    
% Application point of the contact force on the left foot w.r.t. CoM
Pl             = pos_leftFoot - xCoM;     

AL             = [eye(3)  , zeros(3);
                  skew(Pl), eye(3)];
               
AR             = [eye(3)  , zeros(3);
                  skew(Pr), eye(3)];  
                
% Velocity of the center of mass 
xCoM_dot       = J_CoM(1:3,:) * nu;                   
                
% Both AR_dot = AL_dot = A_dot 
%                      = [0             , 0 
%                        -skew(xCoM_dot), 0]

A_dot           = [ zeros(3)       , zeros(3);
                   -skew(xCoM_dot) , zeros(3)];   
               
% left and right wrenches readings from the F/T sensors
f_ext_L         = Left_Right_F_T_Sensors(1:6);
f_ext_R         = Left_Right_F_T_Sensors(7:end);               

% derivatives of the left and right parametrized forces (computed in
% parametrized forces matlab function block)
% F_L           = d(f_L)/d(xi_L)
% F_R           = d(f_R)/d(xi_R)
F_L             =  F(1:6, 1:6);
F_R             =  F(1:6, 7:12);       

A_total         = [AL*F_L, AR*F_R];

pinvA_total     =  pinv(A_total, Reg.pinvDamp)* constraints(1) * constraints(2)  ...
                   + [inv(AL*F_L); zeros(6)]  * constraints(1) *(1-constraints(2)) ... 
                   + [zeros(6) ; inv(AR*F_R)] * constraints(2) *(1-constraints(1));  
                  
nullA_total     = (eye(12,12)-pinvA_total*A_total)*constraints(1)*constraints(2);

%% Acceleration of the center of mass 
% dot(H)   = A*f_external + mge3  

H_dot           = AL*f_ext_L*constraints(1) + AR*f_ext_R*constraints(2) + gravityWrench;
xCoM_ddot       = H_dot(1:3,:)/m;
w_dot           = H_dot(4:6,:);

%% Desired momentum acceleration dynamics 
if abs((xCoM - desired_x_dx_ddx_CoM(:,1))) < 0.001
    %gainsPCOM(3) = 0;
    %gainsICOM(3) = 0;
    %gainsDCOM(3) = 0;
end

xCoM_Jerk_Star_linear  = -gainsPCOM.*(xCoM_dot - desired_x_dx_ddx_CoM(:,2)) ...
                         -gainsICOM.*(xCoM - desired_x_dx_ddx_CoM(:,1))     ...
                         -gainsDCOM.*(xCoM_ddot - desired_x_dx_ddx_CoM(:,3));
                     
xCoM_Jerk_Star_angular = -Gain.KP_AngularMomentum(((state-1)*3)+1:((state-1)*3)+3 ,:)*H(4:end) ...
                         -Gain.KI_AngularMomentum*intHw                                        ...
                         -Gain.KD_AngularMomentum(((state-1)*3)+1:((state-1)*3)+3 ,:)*w_dot;
                     
H_ddot_star            = [m*xCoM_Jerk_Star_linear;
                          xCoM_Jerk_Star_angular];   
              
Beta                   =  A_dot*f_ext_L*constraints(1) + A_dot*f_ext_R*constraints(2);

%% xi_dot realizing the desired CoM jerk dynamics

%Minimizing joint torques null space
if constraints(1) == 1 && constraints (2) == 1 
     xi_dot1       = pinvA_total * (H_ddot_star - Beta);
     xi_dot0       = -pinvDamped((Sigma*nullA_total), Reg.pinvDamp) ...
                       * ((Sigma * xi_dot1) + Gain.k_t*tau);
                   
     xi_dot        =  xi_dot1 + nullA_total * xi_dot0 ;  
         fprintf('two feet balacning')
    else
     xi_dot        = pinvA_total * (H_ddot_star - Beta);      
         fprintf('one foot yoga')
end

% CoM transistion on left foot null space
%  k_xi = diag(0*[1 1 1 1 1 1 1 1 1 1 1 1]);
%  xi_dot1 = pinvA_total * (H_ddot_star - Beta);
%  xi_desired = [0 0.06508 log(300) 0.1774 0.0025 0 0 0 0 0 0 0]';
%     
% if state == 2
%         xi_dot        =  xi_dot1 + nullA_total * k_xi *(xi_desired - xi) ;  
%     else
%         xi_dot = xi_dot1;
% end
%% DEBUG DIAGNOSTICS

% Error on the center of mass
errorCoM              = xCoM - desired_x_dx_ddx_CoM(:,1);

% Error on the center of mass velocity
errorCoM_dot          = xCoM_dot - desired_x_dx_ddx_CoM(:,2);

% Error on the center of mass acceleration
errorCoM_ddot         = xCoM_ddot - desired_x_dx_ddx_CoM(:,3);

% Error on the center of mass angular velocity
errorCoM_dot_angular  = 0 - H(4:end);

% Error on the center of mass angular acceleration
errorCoM_ddot_angular = 0 - w_dot;

% Error on the linear momentum
error_H_linear        = m*(desired_x_dx_ddx_CoM(:,2) - xCoM_dot);

% Error on the angular momentum
error_H_angular       = 0 - H(4:end);

% Linear momentum derivative
HDot_linear_ft        = H_dot(1:3,:);
    
%Angular momentum derivative 
HDot_angular_ft       = H_dot(4:6,:);
end