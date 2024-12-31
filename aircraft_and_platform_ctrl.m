function [sys,x0,str,ts] = spacemodel(t,x,u,flag)
switch flag ,
    case 0 ,
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 3 ,
        sys = mdlOutputs(t,x,u);
    case {2, 4, 9}
        sys = [];
    otherwise
        error(['Unhandled flag = ', num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes
    sizes = simsizes;
    sizes.NumContStates = 0;    % No continuous states
    sizes.NumDiscStates = 0;    % No discrete states
    sizes.NumOutputs = 1;       % One output (control signal)
    sizes.NumInputs = 5;        % Five inputs
    sizes.DirFeedthrough = 1;   % Direct feedthrough
    sizes.NumSampleTimes = 1;   % Sample time
    sys = simsizes(sizes);
    x0 = [];
    str = [];
    ts = [0 0];
 
    function sys = mdlOutputs(t,x,u)
   
  xd = 250 * exp(-0.2839 * t);                     % Desired position
  dxd = -0.2839 * 250 * exp(-0.2839 * t);          % First derivative (velocity)
  ddxd = (0.2839)^2 * 250 * exp(-0.2839 * t);        % Second derivative (acceleration)
  dddxd = -(0.2839)^3 * 250 * exp(-0.2839 * t);        % Third derivative (jerk)
  ddddxd = (0.2839)^4 * 250 * exp(-0.2839 * t);      % Fourth derivative

% Reference signals and their derivatives
%     xd = 70*sin(t);           % Desired position
%     dxd = 70*cos(t);          % First derivative (velocity)
%     ddxd = -70*sin(t);        % Second derivative (acceleration)
%     dddxd = -70*cos(t);       % Third derivative (jerk)
%     ddddxd = 70*sin(t);       % Fourth derivative
    
    % System states
    x1 = u(2);
    x2 = u(3);
    x3 = u(4);
    x4 = u(5);

    % Control gains
    c1 = 27; 
    c2 = 27; 
    c3 = 9;
    
    % System constants
    l = 1; 
    g = 9.8;

    % Dynamics
    f1 = g * sin(x1) / l + x3;     % System dynamic equation
    f2 = 0;
    b = 1;                         % Control gain

    % Partial derivatives for sliding mode control
    f1_x1 = g * cos(x1) / l;
    f1_x2 = 0;
    f1_x3 = 1;
    beta3 = 1.0 + 0.1;
    % Disturbance term D
    D = -(g/l) * sin(x1) * x2^2 + (g/l) * cos(x1) * f1;
    
    % Equivalent control (ueq)
    ueq = -inv(f1_x3 * b)*(c1*x2 + c2*f1 + c3*f1_x1*x2 + c3*f1_x2*f1 + c3*f1_x3*x4 + D - c1*dxd - c2*ddxd - c3*dddxd - ddddxd);

    % Errors
    e1 = x1 - xd;
    e2 = x2 - dxd;
    e3 = f1 - ddxd;
    e4 = f1_x1*x2 + f1_x2*f1 + f1_x3*x4 - dddxd;
    
    % Sliding surface
    s = c1*e1 + c2*e2 + c3*e3 + e4;
    
    % Saturation function to reduce chattering
    
    d_up = 10;
    rou = 1.0;
    M = beta3 * d_up + rou;
    nmn = 1.0;  % Nonlinear switching gain
    
    % Saturation function
    S = 2;  % Choose between different modes (S = 1 for sign, S = 2 for saturation)
        
    if S == 1
        sat = sign(s);
    elseif S == 2  % Saturated function
        fai = 0.05;  % Boundary layer width
        if abs(s) <= fai
            sat = s / fai;
        else
            sat = sign(s);
        end
    end
    
    % Sliding mode control (usw)
    usw = -(M * sat + nmn * s) / (f1_x3 * b);
    
    % Total control input
    ut = ueq + usw;
    
    % Output the control signal
    sys(1) = ut;