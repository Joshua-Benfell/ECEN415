% Nonlinear rocket simulation.

%==============================================================================
svnURL    = '$URL: svn+ssh://regent.ecs.vuw.ac.nz/home/hollitch/Base/Repository/Teaching/ECEN415_Advanced_Control/Trunk/Assignments/tst/Assignment_2_2021_Q3.m $';
svnRev    = '$Rev: 554 $';
svnAuthor = '$Author: hollitch $';
svnDate   = '$Date: 2021-08-23 17:17:07 +1200 (Mon, 23 Aug 2021) $';
svnId     = '$Id: Assignment_2_2021_Q3.m 554 2021-08-23 05:17:07Z hollitch $';
SRL = 0;

%==============================================================================

clear
global g c_d M J L eta k

g= -9.8;   
c_d = 2e-3; 
M = 1000;
J = 20e3;
L = 5;
eta = 1000;
k = 6;

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

% f = @(t,x) [x(2); 
%             g + x(8)*cos(deg2rad(x(5))) / (M+x(7)) - c_d*x(2)*abs(x(2)); 
%             x(4);
%             x(8)*sin(deg2rad(x(5))) / (M+x(7));
%             x(6);
%             k/(J+L^2*x(7))*x(9);
%             -1/eta*(x(8)+x(9));
%             0;
%             0];

% This includs a better drag model for X(1) < 12km.
% https://en.wikipedia.org/wiki/Density_of_air (exponential approximation).
% c_d is chosen to give a terminal veolocity of about 90 m/s at ground
% level.
f = @(t,x) [x(2); 
            g + x(8)*cos(deg2rad(x(5))) / (M+x(7)) - c_d*x(2)*abs(x(2))*exp(-x(1)/10400); 
            x(4);
            x(8)*sin(deg2rad(x(5))) / (M+x(7));
            x(6);
            k/(J+L^2*x(7))*x(9);
            -1/eta*(x(8)+x(9));
            0;
            0];
        
%==========================================================================

% Simulation parameters ---------------------------------------------------
global x_0
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

syms x1 x2 x3 x4 x5 x6 x7 i1 i2
f_sym = @(t,x) [x2; 
                g + i1*cos(x5) / (M+x7) - c_d*x2^2;  
                x4;
                i1*sin(x5) / (M+x7);
                x6;
                k/(J+L^2*x7)*i2;
                -1/eta*(i1+i2);
                ];
            

% 
% The simulation ----------------------------------------------------------
store = [];
t_store = [];

figure(1)
clf;
while (x(1) > -0.01) & (elapsed_time <  15)
    if x(7) >  0 % Check whether there is any fuel remaining.
    [u1, u2] = controller_command(elapsed_time, x);
    else
        u1 = 0;
        u2 = 0;
    end
  
  % Add some wind disturbances to the rocket.
  %w1 = randn(1,1) * 2000*exp(-(1+x(1))/10400);
  %w2 = randn(1,1) * 1000*exp(-(1+x(1))/10400);
    
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
 
figure(2)
subplot(211)
plot(t_store, x_store)
ylabel('x')

subplot(212)
plot(t_store, u_store)
xlabel('Time [s]')
ylabel('u')

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

%--------------------------------------------------------------------------
% Controller code
%--------------------------------------------------------------------------
% You should change the content of this function. The example code included
% here is not particularly clever, but should run.

function[u1, u2] = controller_command(t, x)
    global eta x_0 g M c_d J L k
    
    if (length(x) == 7)
       x(8:9) = [0 0];
    end
    
    syms x1 x2 x3 x4 x5 x6 x7 i1 i2
    f_sym = @(t,x) [x2; 
                g + i1*cos(x5) / (M+x7) - c_d*x2^2;  
                x4;
                i1*sin(x5) / (M+x7);
                x6;
                k/(J+L^2*x7)*i2;
                -1/eta*(i1+i2);
                ];
    df = jacobian(f_sym, [x1 x2 x3 x4 x5 x6 x7]);
    dg = jacobian(f_sym, [i1 i2]);
    A = double(subs(df, [x1 x2 x3 x4 x5 x6 x7 i1 i2], x));
    B = double(subs(dg, [x5 x7], [x(5) x(7)]));
    C = [1 1 1 1 1 1 1];
    D = 0;
    
    sys = ss(A,B,C,D);
    
    
    %K = place(A, B, p)
    
    %x_f = [1500, nan, 100, 20, 10, 0, 0];
    
    u1 = eta*x_0(7)/15; % Burn all of the fuel in 15s at uniform rate. 
    u2 = eta*2;
end

% END =========================================================================