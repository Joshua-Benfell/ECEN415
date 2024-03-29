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

K_shrink = place(A_shrink,B_shrink,p);
K = zeros(size(B.'));
K(1:2,1:2) = K_shrink

reg_sys = ss(A-B*K, B, C, D)


p = [-100+500j -100-500j -200+500j -200-500j -100 -100];
k_adv = place(A, B, p);
sys_adv = ss(A-B*k_adv, B, C, D)

figure(1)
impulse(sys)
hold on
impulse(reg_sys)
impulse(sys_adv)
hold off
legend("Unregulated", "Simplified Regulated", "Regulated")

% c integrators
A_i = [A zeros(length(A),1) zeros(length(A),1) ; -1 0 0 0 0 0 0 0; 0 -1 0 0 0 0 0 0];
B_i = [B ; 0 0; 0 0];

alpha_c = [p -150 -150];

k_i = place(A_i, B_i, alpha_c);
sys_i = ss(A_i-B_i*k_i, [zeros(size(B)); 1 0; 0 1], [C 0 0], D);

A_perturbed = [-50 0 -90 355 -205 0;
                 0 10 -50 125 -375 1500;
                 0 0 -100 200 -350 500;
                 0 0 -600 150 200 200;
                 0 0 500 -350 -400 500;
                 0 0 0 -250 -650 -200;
                ];
A_i_perturbed = [A_perturbed zeros(length(A_perturbed),1) zeros(length(A_perturbed),1) ; -1 0 0 0 0 0 0 0; 0 -1 0 0 0 0 0 0];
sys_i_perturbed = ss(A_i_perturbed-B_i*k_i, [zeros(size(B)); 1 0; 0 1], [C 0 0], D);


figure(2)
step(sys)
hold on
step(reg_sys)
step(sys_adv)
step(sys_i)
step(sys_i_perturbed)
hold off
legend("Unregulated", "Simplified Regulated", "Regulated", "Integrator", "Perturbed Integrator")

% d 
x0 = [1 1 0 1 0 0 -1 0];
t = linspace(0,1,10000).';
u = [repmat([0 0], length(t)/4, 1);
    repmat([1 1], length(t)/4, 1);
    repmat([2 1], length(t)/4, 1);
    repmat([0 0], length(t)/4, 1);];

figure(3)
lsim(sys_i, u, t, x0)