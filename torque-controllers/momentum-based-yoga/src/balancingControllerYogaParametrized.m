%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [tau_star, errorCoM, error_H_linear,  error_H_angular, f_desired, xi_dot]    =  ...
              balancingControllerYogaParametrized(SM_TYPE_BIN, constraints, ROBOT_DOF_FOR_SIMULINK, qj, qjDes, nu, M, h, L, ...
                                      intHw, w_H_l_sole, w_H_r_sole, JL, JR, dJL_nu, dJR_nu, xCoM, ...
                                      J_CoM, desired_x_dx_ddx_CoM, gainsPCOM, gainsDCOM, gainsICOM, ...
                                      impedances, Gain, Reg, Left_Right_F_T_Sensors, f, F, xi, state)
    %% DEFINITION OF CONTROL AND DYNAMIC VARIABLES
    pos_leftFoot   = w_H_l_sole(1:3,4);
%   w_R_l_sole     = w_H_l_sole(1:3,1:3);
    pos_rightFoot  = w_H_r_sole(1:3,4);
%   w_R_r_sole     = w_H_r_sole(1:3,1:3);

    dampings       = Gain.dampings;
    ROBOT_DOF      = size(ROBOT_DOF_FOR_SIMULINK,1);
    gravAcc        = 9.81;
    
    % Mass of the robot
    m              = M(1,1);
    
    % The mass matrix is partitioned as:
    %
    %   M = [ Mb,   Mbj
    %         Mbj', Mj ];  
    %
    % where: Mb  \in R^{6x6}
    %        Mbj \in R^{6x6+nDof}
    %        Mj  \in R^{nDofxnDof}
    %
    Mb             = M(1:6,1:6);
    Mbj            = M(1:6,7:end);
    Mj             = M(7:end,7:end);

    St             = [zeros(6,ROBOT_DOF);
                      eye(ROBOT_DOF,ROBOT_DOF)];
    gravityWrench  = [zeros(2,1);
                     -m*gravAcc;
                      zeros(3,1)];
                  
    % Application point of the contact force on the right foot w.r.t. CoM
    Pr             = pos_rightFoot - xCoM; 
    
    % Application point of the contact force on the left foot w.r.t. CoM
    Pl             = pos_leftFoot - xCoM;     
    
    % Velocity of the center of mass 
    xCoM_dot       = J_CoM(1:3,:) * nu;              
                         
    % The following variables serve for determening the rate-of-change and 
    % double rate-of-change of the robot's momentum.
    % In particular, when balancing on two feet, one has:F1
    %
    %   dot(H) = gravityWrench +  AL*f_L + AR*f_R
    %          = gravityWrench + [AL,AR]*f
    %
    % where  f_L and f_R are the contact wrenches acting on the left and
    % right foot, respectively, and f = [f_L;f_R].
    %
    AL             = [ eye(3),zeros(3);
                       skew(Pl), eye(3)];
    AR             = [ eye(3), zeros(3);
                       skew(Pr), eye(3)];                             
                    
    % dot(H) = mg + A*f
    A              = [AL, AR]; 
    
    pinvA          = pinv(A, Reg.pinvTol)*constraints(1)*constraints(2)  ...
                      + [pinv(AL);zeros(6)]*constraints(1)*(1-constraints(2)) ... 
                      + [zeros(6);pinv(AR)]*constraints(2)*(1-constraints(1)); 
                  
    % Both AR_dot = AL_dot = A_dot = [0             , 0 
    %                                -skew(xCoM_dot), 0]                             
    A_dot           = [ zeros(3)       , zeros(3);
                       -skew(xCoM_dot) , zeros(3)];               
                
    % Null space of the matrix A            
%   NA              = (eye(12,12)-pinvA*A)*constraints(1)*constraints(2);              
                      
    % left and right wrenches readings from the F/T sensors
    f_ext_L         = Left_Right_F_T_Sensors(1:6);
    f_ext_R         = Left_Right_F_T_Sensors(7:end);
    
    % derivatives of the left and right parametrized forces (computed in
    % parametrized forces matlab function block)
    % F_L           = d(f_L)/d(xi_L)
    % F_R           = d(f_R)/d(xi_R)
    F_L             =  F(1:6, 1:6);
    F_R             =  F(1:6, 7:12);
    
    % Acceleration of the center of mass 
    % dot(L)   = A*f_external + mge3  
    L_dot           = AL*f_ext_L*constraints(1) + AR*f_ext_R*constraints(2) + gravityWrench;
    xCoM_ddot       = L_dot(1:3,:)/m;
    w_dot           = L_dot(4:6,:);
    
    % ddot(H) = dot(A)*f + A_R*F_R*dot(xi_R) + A_L*F_L*dot(xi_L) 
    A_total         = [AL*F_L, AR*F_R];
                  
    pinvA_total     =  pinv(A_total, Reg.pinvDamp) * constraints(1) * constraints(2)  ...
                      + [pinv(AL*F_L, Reg.pinvTol); zeros(6)]  * constraints(1) * (1-constraints(2)) ... 
                      + [zeros(6) ; pinv(AR*F_R, Reg.pinvTol)] * constraints(2) *(1-constraints(1)); 
                  
    nullA_total     = (eye(12,12)-pinvA_total*A_total)*constraints(1)*constraints(2);
                        
    % Joint velocity
    qjDot          = nu(7:end);
    
    % Joint position error
    qjTilde        = qj - qjDes;
    
    % Desired acceleration for the center of mass 
    % xDDcomStar   = desired_x_dx_ddx_CoM(:,3) - gainsPCOM.*(xCoM - desired_x_dx_ddx_CoM(:,1)) -gainsDCOM.*(xCoM_dot - desired_x_dx_ddx_CoM(:,2));
    
    %Desired jerk for the center of mass (used in global force parametrization controller)
    xCoM_Jerk_Star = -gainsPCOM.*(xCoM_dot - desired_x_dx_ddx_CoM(:,2)) - gainsICOM.*(xCoM - desired_x_dx_ddx_CoM(:,1)) - gainsDCOM.*(xCoM_ddot - desired_x_dx_ddx_CoM(:,3));

    % Time varying contact jacobian
    Jc             = [JL*constraints(1);      
                      JR*constraints(2)];
                   
    % Time varying dot(J)*nu
    Jc_nuDot       = [dJL_nu*constraints(1) ;      
                      dJR_nu*constraints(2)];

    JcMinv         = Jc/M;
    JcMinvSt       = JcMinv*St;
    JcMinvJct      = JcMinv*transpose(Jc);
    
    % multiplier of f in tau
    JBar           = transpose(Jc(:,7:end)) -Mbj'/Mb*transpose(Jc(:,1:6)); 

    Pinv_JcMinvSt  = pinvDamped(JcMinvSt,Reg.pinvDamp); 
   
    % nullJcMinvSt --> null space of Pinv_JcMinvSt
    nullJcMinvSt   = eye(ROBOT_DOF) - Pinv_JcMinvSt*JcMinvSt;

    % Mbar is the mass matrix associated with the joint dynamics, i.e.
    Mbar           = Mj-Mbj'/Mb*Mbj;
    NLMbar         = nullJcMinvSt*Mbar;
    
    % Adaptation of control gains for back compatibility with older
    % versions of the controller
    impedances     = diag(impedances)*pinv(NLMbar,Reg.pinvTol) + Reg.impedances*eye(ROBOT_DOF);
    dampings       = diag(dampings)*pinv(NLMbar,Reg.pinvTol)  + Reg.dampings*eye(ROBOT_DOF); 
   
    % IROS_2016
    % Sigma collects all the terms in tau to be multiplied by f
    % tauModel collects the rest of the terms with the pd joint controller
    % to follow a desired postural task
    Sigma          = -(Pinv_JcMinvSt*JcMinvJct + nullJcMinvSt*JBar);
    tauModel       =  Pinv_JcMinvSt*(JcMinv*h - Jc_nuDot) + nullJcMinvSt*(h(7:end) - Mbj'/Mb*h(1:6) ...
                     -impedances*NLMbar*qjTilde -dampings*NLMbar*qjDot);
    
    % Desired rate-of-change of the robot momentum 
    % HDotDes      = [m*xDDcomStar ;
    %                -Gain.KD_AngularMomentum*L(4:end) - Gain.KP_AngularMomentum*intHw];
    
    %% Desired momentum jerk dynamics 
     %   L_ddot_star    = [m*xCoM_Jerk_Star;
      %                   -Gain.KP_AngularMomentum*L(4:end) - Gain.KD_AngularMomentum*w_dot - Gain.KI_AngularMomentum *intHw] ;          
        
    L_ddot_star    = [m*xCoM_Jerk_Star;
                      -Gain.KP_AngularMomentum(((state-1)*3)+1:((state-1)*3)+3 ,:)*L(4:end) - Gain.KD_AngularMomentum(((state-1)*3)+1:((state-1)*3)+3 ,:)*w_dot - Gain.KI_AngularMomentum*intHw] ;          
   

    Beta           =  A_dot*f_ext_L*constraints(1) + A_dot*f_ext_R*constraints(2);
     
    
    %% joint torques realizing the desired xi_dot
    
     tau       = Sigma*f + tauModel;
    
    %% xi_dot option for minimizing the joint torques
    
   if  constraints(1) == 1 && constraints (2) == 1
         xi_dot1       = pinvA_total * (L_ddot_star - Beta);
         xi_dot0       = -pinvDamped((Sigma*nullA_total), Reg.pinvDamp) ...
                       * ((Sigma * xi_dot1) + Gain.k_t*tau);
                   
         xi_dot        =  xi_dot1 + nullA_total * xi_dot0 ;  
         fprintf('two feet balacning')
    else
         xi_dot        = pinvA_total * (L_ddot_star - Beta);      
         fprintf('one foot yoga')
    end                   
    tau_star      = tau;
    %% DEBUG DIAGNOSTICS

    % Desired parametrized contact wrenches
     f_desired     = f;
    
    % Error on the center of massF1
    errorCoM       = xCoM - desired_x_dx_ddx_CoM(:,1);
    
    % Error on the linear momentum
    error_H_linear = m*(desired_x_dx_ddx_CoM(:,2) - xCoM_dot);
    
    % Error on the angular momentum
    error_H_angular= 0 - L(4:end);
end
