% Nonlinear rocket simulation.
% Version 554
% 23/8/2021
% C.Hollitt

%==============================================================================

clear
close all
g= -9.8;    % [m/s^2]   
c_d = 2e-3; % [Ns^2/m^2]
M = 1000;   % [kg]
J = 20e3;   % [kg m^2]
L = 5;      % [m]
eta = 1000; % [Ns/kg]
k = 6;      % [m]

% =========================================================================
% Nonlinear model of the rocket motion.
% =========================================================================
% The state here is defined as in the assignment.
% Note that u_1 is passed as x(8) and u_2 is passed to the solver as x(9)
% (That is, matlab's ODE solver doesn't have separate inputs, so we must
% pass them as elements of the x vector.)
%
% x(2)*abs(x(2)) is used in the drag equation to make sure that it always
% opposes the direction of motion. Note however, that it should actually
% not act only on the vertical velocity, but the total velocity.
%
% Note that the model does not check for ground contact, so will allow the
% rocket to move to negative altitude.

f = @(t,x) [x(2); 
            g + x(8)*cos(deg2rad(x(5))) / (M+x(7)) - c_d*x(2)*abs(x(2)); 
            x(4);
            x(8)*sin(deg2rad(x(5))) / (M+x(7));
            x(6);
            k/(J+L^2*x(7))*x(9);
            -1/eta*(x(8)+x(9));
            0;
            0];
        
%==========================================================================

% Simulation parameters ---------------------------------------------------
global x_0 % Makes x_0 accessible in plot_telemetry function.
x_0 = [0   % Altitude [m]
       0;  % Vertical speed [m/s]
       0;  % Horizontal position [m]
       0;  % Horizontal speed [m/s]
       0;  % Rocket angle [deg]
       0;  % Rocket rotational speed [deg/s]
       1000;  % Fuel remaining [kg]
       ]';
x = x_0;
      
dt = 1;
elapsed_time = 0;


% % Symbolic Model --------------------------------------------------------

syms x1 x2 x3 x4 x5 x6 x7 u1 u2
f_sym = @(t,x) [x2; 
                g + u1*cos(x5) / (M+x7) - c_d*x2^2;  
                x4;
                u1*sin(x5) / (M+x7);
                x6;
                k/(J+L^2*x7)*u2;
                -1/eta*(u1+u2);
                ];
            
% The simulation ----------------------------------------------------------
store = [];
t_store = [];

figure(1)
clf;
while elapsed_time <  15
    if x(7) >  0 % Check whether there is any fuel remaining.
        
        %******************************************************************
        % YOUR CONTROL CODE GOES HERE 
        % Set the u vector as you wish.
        u1 = 66000;
        if elapsed_time < 5
            u2 = 0;
        else
            u2 = 1025;
        end
        %******************************************************************
        
    else
        u1 = 0;
        u2 = 0;
    end
  
  x(8) = u1; % Load the inputs as "fake" state variables for the solver.
  x(9) = u2; 
  
  [t,x] = ode45(f, [0 dt], x);
  
  plot_telemetry(elapsed_time+t, x);

  % You can use these variables later to examine the flight.
  t_store = [t_store; elapsed_time+t];
  store = [store; x];
  
  elapsed_time = elapsed_time + dt
  x = x(end,:);
end

x_store = store(:,1:7);
u_store = store(:,8:9);


%--------------------------------------------------------------------------
% Linearisation from t = 5 (When it starts pitching over) 3d
%--------------------------------------------------------------------------
syms u1 u2
df = jacobian(f_sym, [x1 x2 x3 x4 x5 x6 x7]);
dg = jacobian(f_sym, [u1 u2]);
A = double(subs(df, [x1 x2 x3 x4 x5 x6 x7 u1 u2], [x_store(230,1:7) 66000 0]))
B = double(subs(dg, [x5 x7], [x_store(230, 5) x_store(230, 7)]));


elapsed_time = 0;
figure(2)
t = 0:0.025:15;
len_t = length(t);
% set up input Matrix
u = zeros(2,len_t);
u(1,:) = 66000 * ones(1, len_t);
u(2, t>=5) = 1025;

sys = ss(A,B,[],[]);

[Y, Tsim, X] = lsim(sys, u, t, x_0);
X(:,8:9) = u.';
plot_telemetry(Tsim, X);


% End Linearisation 1 -----------------------------------------------------

%--------------------------------------------------------------------------
% Linearisation from t = 15 (When it starts pitching over) 3e
%--------------------------------------------------------------------------
syms u1 u2
A = double(subs(df, [x1 x2 x3 x4 x5 x6 x7 u1 u2], [x_store(end,1:7) 66000 0]));
B = double(subs(dg, [x5 x7], [x_store(end, 5) x_store(end, 7)]));


elapsed_time = 0;
figure(3)
t = 0:0.025:15;
len_t = length(t);
% set up input Matrix
u = zeros(2,len_t);
u(1,:) = 66000 * ones(1, len_t);
u(2, t>=5) = 1025;

sys = ss(A,B,[],[]);

[Y, Tsim, X] = lsim(sys, u, t, x_0);
X(:,8:9) = u.';
plot_telemetry(Tsim, X);


% End Linearisation 2 -----------------------------------------------------

%--------------------------------------------------------------------------
% Plotting function to display the telemetry during flight.
%--------------------------------------------------------------------------
function[] = plot_telemetry(t,x)
  global x_0

  subplot(1,2,1)
  xlabel('Distance downrange [m]')
  ylabel('Altitude [m]')
  hold on;
  plot(x(:,3), x(:,1), 'b')
  
  subplot(4,2,4)
  xlabel('Time [s]')
  ylabel('Velocities [m/s]')
  hold on
  plot(t, x(:,2), 'b', t, x(:,4), 'r')
  legend('Vertical speed', 'Horizontal speed', 'Location', 'northwest')
  
  subplot(4,2,6)
  xlabel('Time [s]')
  ylabel('Pitch angle [deg]')
  hold on
  plot(t, x(:,5), 'b')
  
  subplot(4,2,8)
  xlabel('Time [s]')
  ylabel('Fuel [kg]')
  hold on
  plot(t, x(:, 7), 'b')
  if x(:,7)<0
    set(gca, 'Color' ,'r')
  end
  
  subplot(4,2,2)
  plot(0,0,'bo')
  box off
  hold on
  axis equal
  axis off
  max_u1 = 1000/0.001/15;
  line([0, max_u1*sin(deg2rad(x(end,5)))], [0 max_u1*cos(deg2rad(x(end,5)))],'Color','b')
  line([0, -x(end,8)*sin(deg2rad(x(end,5)))], [0 -x(end,8)*cos(deg2rad(x(end,5)))],'Color','r')
  plot(-x(end,8)*sin(deg2rad(x(end,5))),-x(end,8)*cos(deg2rad(x(end,5))),'r*')
  hold off
  
  drawnow
  
end

% END =========================================================================
