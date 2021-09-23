close all
clear

A = [-10 0 -90 295 -205 0;
     0 -50 -50 125 -375 1000;
     0 0 -100 200 -300 500;
     0 0 -500 100 200 500;
     0 0 500 -300 -400 500;
     0 0 0 -250 -250 -200;
    ];

B = [1 0;
     0 2;
     0 0;
     0 0;
     0 0;
     0 0;
     ];
 
C = [1 1 -2 0 0 0];

D = [0 0];

sys = ss(A, B, C, D);

% a - non-Controllable but Stabilisable
Co = ctrb(A,B)
rank_Co = rank(Co)
%x1 and x2 are controllable
csys = canon(sys,'modal')
% All are stable as all have a -ive exponential decay, so go to 0

% b
% Slowest is -100+-500j
p = [-100 -100];

A_shrink = csys.A(1:2,1:2);
B_shrink = csys.B(1:2,1:2);
C_shrink = csys.C(1:2);
D_shrink = csys.D;

K_shrink = place(A_shrink,B_shrink,p);
K = zeros(size(B.'));
K(1:2,1:2) = K_shrink

reg_sys = ss(A-B*K, B, C, D)
impulse(sys)
hold on
impulse(reg_sys)
hold off

% c integrators