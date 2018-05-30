function [Sigma, tauModel]= IROS2016(ROBOT_DOF_FOR_SIMULINK, JL, JR, dJL_nu, dJR_nu, h, impedances, qj, qjDes, constraints, nu, M, Gain, Reg)

ROBOT_DOF      = size(ROBOT_DOF_FOR_SIMULINK,1);

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
    
    %Torque selector matrix
    St             = [zeros(6,ROBOT_DOF);
                      eye(ROBOT_DOF,ROBOT_DOF)];
   
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
    dampings       = Gain.dampings;
    dampings       = diag(dampings)*pinv(NLMbar,Reg.pinvTol)   + Reg.dampings*eye(ROBOT_DOF); 
   
    % Joint velocity
    qjDot          = nu(7:end);
    
    % Joint position error
    qjTilde        = qj - qjDes;
    
    %% IROS_2016
    % Sigma collects all the terms in tau to be multiplied by f
    % tauModel collects the rest of the terms with the pd joint controller
    % to follow a desired postural task
    
    Sigma          = -(Pinv_JcMinvSt*JcMinvJct + nullJcMinvSt*JBar);
    tauModel       =  Pinv_JcMinvSt*(JcMinv*h - Jc_nuDot) + nullJcMinvSt*(h(7:end) - Mbj'/Mb*h(1:6) ...
                     -impedances*NLMbar*qjTilde -dampings*NLMbar*qjDot);
end                