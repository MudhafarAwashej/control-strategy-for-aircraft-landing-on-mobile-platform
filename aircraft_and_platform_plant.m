function [sys,x0,str,ts] = s_function(t,x,u,flag)
switch flag,
    case 0,
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1,
        sys = mdlDerivatives(t,x,u);
    case 3,
        sys = mdlOutputs(t,x,u);
    case {2, 4, 9}
        sys = [];
    otherwise
        error(['Unhandled flag = ', num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes
    sizes = simsizes;
    sizes.NumContStates = 4;  % 4 continuous states
    sizes.NumDiscStates = 0;  % No discrete states
    sizes.NumOutputs = 4;     % 4 outputs (system states)
    sizes.NumInputs = 1;      % 1 input (control signal)
    sizes.DirFeedthrough = 0; % No direct feedthrough
    sizes.NumSampleTimes = 0; % 1 sample time (continuous system)
    sys = simsizes(sizes);
    x0 = [0.15; 0; 0; 0];    % Initial state [x1, x2, x3, x4]
    str = [];
    ts = [];               % Continuous sample time

function sys = mdlDerivatives(t,x,u)
    ut = u(1);                % Control input (u)
    l = 1;                   % Length (system parameter)
    g = 9.8;                  % Gravitational constant
    
    % System dynamics (change this to fit your actual system)
    f1 = g * sin(x(1)) / l + x(3);  % Dynamic equation for x(2)
    
    % State derivatives (dx/dt)
    sys(1) = x(2);            % dx1/dt = x2 (position rate of change)
    sys(2) = f1;              % dx2/dt = f1 (acceleration term)
    sys(3) = x(4);            % dx3/dt = x4 (velocity rate of change)
    sys(4) = ut - 20 * sin(t);% dx4/dt = control input and disturbance

function sys = mdlOutputs(t,x,u)
    % Outputs: return current state variables
    sys(1) = x(1);  % x1 (position)
    sys(2) = x(2);  % x2 (velocity)
    sys(3) = x(3);  % x3 (position)
    sys(4) = x(4);  % x4 (velocity)