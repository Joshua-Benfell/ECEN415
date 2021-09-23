close all
clear

A = [0 1 0;
     0 0 1;
     -18 -15 -2;
     ];
 
B = [0;
     0;
     1
     ];

C = [1 0 0];

D = 0;

controllability_matrix = [B A*B A^2*B];

ts = 0.1; % Sampling Time

% Continuous Time
poles = [-1.33+1.49j, -1.33-1.49j, -13.3];
impulse(ss(A,B,C,D))
K = place(A,B,poles);
hold on
impulse(ss(A-B*K,B,C,D))
hold off

figure(2)
% Discrete Time
sys = ss(A,B,C,D,ts);
impulse(sys)
figure(3)
poles_controlled = exp(poles*ts);
K_controlled = acker(A, B, poles_controlled); % Dead Beat Controller
sys_controlled = ss(A-B*K_controlled, B, C, D, ts);
impulse(sys_controlled)
