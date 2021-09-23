%% a
close all
clear

% Create System
A = [12.5314 , -91.36 , 28.7129 , 59.9628 ; 
     21.316 , -115.6631 , 37.5584 , 75.0519 ; 
     13.967 , -53.5817 , 15.4161 , 33.4391 ; 
     20.5222 , -123.3107 , 42.267 , 79.7156]; %4x4

B = [1.7649 , 1.7649; 
     2.8318 , 2.8318; 
     2.2353 , 2.2353; 
     2.7294 , 2.7294]; %2x4
 
C = [-0.46166 , -4.4635 , 1.9151 , 3.7274 ;
      1.0853 , -12.82 , 4.5753 , 8.3759]; %4x2
  
D = [0 , 0 ; 
     0 , 0 ]; %2x2

sys = ss(A,B,C,D);
% End Create System

T = 0:0.01:10; 
X0 = [1, 1, 1, 2].';  % Column vector of initial state
U = zeros(2, length(T));  % Set up with no inputs 2x1001

[Y, Tsim, X] = lsim(sys, U, T, X0);

figure(1)
plot(Tsim, Y);
title("Q1.a Time Response of Y");
legend("Y(1)", "Y(2)")
figure(2)
plot(Tsim, X);
title("Q1.a Time Response of X");
legend("X(1)", "X(2)", "X(3)", "X(4)")
%% b
%close all
clear

% Create System
A = [12.5314 , -91.36 , 28.7129 , 59.9628 ; 
     21.316 , -115.6631 , 37.5584 , 75.0519 ; 
     13.967 , -53.5817 , 15.4161 , 33.4391 ; 
     20.5222 , -123.3107 , 42.267 , 79.7156]; %4x4

B = [1.7649 , 1.7649; 
     2.8318 , 2.8318; 
     2.2353 , 2.2353; 
     2.7294 , 2.7294]; %2x4
 
C = [-0.46166 , -4.4635 , 1.9151 , 3.7274 ;
      1.0853 , -12.82 , 4.5753 , 8.3759]; %4x2
  
D = [0 , 0 ; 
     0 , 0 ]; %2x2

% End Create System

T = 0:0.01:6;
T_idx = 2:1:length(T);
X0 = [1, 1, 1, 2].';  % Column vector of initial state
X = zeros(4,length(T));
X(:,1) = X0;


% As there is no input, it is autonomous, so we can ignore B and D

for t = T_idx
    X(:,t) = expm((T(t) - T(t-1))*A)*X(:,(t-1));
end

figure(3)
plot(T,X)
title("Q1.b Exponentially solved time response of X")
legend("X(1)", "X(2)", "X(3)", "X(4)")
Y = C * X;
figure(4)
plot(T,Y)
title("Q1.b Exponentially solved time response of Y")
legend("Y(1)", "Y(2)")
%% c
clear
% Create System
A = [12.5314 , -91.36 , 28.7129 , 59.9628 ; 
     21.316 , -115.6631 , 37.5584 , 75.0519 ; 
     13.967 , -53.5817 , 15.4161 , 33.4391 ; 
     20.5222 , -123.3107 , 42.267 , 79.7156]; %4x4

B = [1.7649 , 1.7649; 
     2.8318 , 2.8318; 
     2.2353 , 2.2353; 
     2.7294 , 2.7294]; %2x4
 
C = [-0.46166 , -4.4635 , 1.9151 , 3.7274 ;
      1.0853 , -12.82 , 4.5753 , 8.3759]; %4x2
  
D = [0 , 0 ; 
     0 , 0 ]; %2x2

sys = ss(A,B,C,D);
 
% End Create System

T = 0:0.01:10; 
X0 = [1, 1, 1, 2].';  % Column vector of initial state
U = [heaviside(T-1);
     2*heaviside(T-5)];  % Set up with heaviside inputs

[Y, Tsim, X] = lsim(sys, U, T, X0);

figure(5)
plot(Tsim,X)
title("Q1.c Time Response of X with Heaviside Step Functions");
legend("X(1)", "X(2)", "X(3)", "X(4)")
figure(6)
plot(Tsim,Y)
title("Q1.c Time Response of Y with Heaviside Step Functions");
legend("Y(1)", "Y(2)")
%% d
clear

% Create System
A = [12.5314 , -91.36 , 28.7129 , 59.9628 ; 
     21.316 , -115.6631 , 37.5584 , 75.0519 ; 
     13.967 , -53.5817 , 15.4161 , 33.4391 ; 
     20.5222 , -123.3107 , 42.267 , 79.7156]; %4x4

B = [1.7649 , 1.7649; 
     2.8318 , 2.8318; 
     2.2353 , 2.2353; 
     2.7294 , 2.7294]; %2x4
 
C = [-0.46166 , -4.4635 , 1.9151 , 3.7274 ;
      1.0853 , -12.82 , 4.5753 , 8.3759]; %4x2
  
D = [0 , 0 ; 
     0 , 0 ]; %2x2

sys = ss(A,B,C,D);
timeStep = 0.001;
T = 0:timeStep:10; 
% End Create System
sysd = c2d(sys, timeStep);

X0 = [1, 1, 1, 2].';  % Column vector of initial state
U = zeros(2, length(T));  % Set up with no inputs 2x1001

[Y, Tsim, X] = lsim(sysd, U, T, X0);
figure(7)
stairs(Tsim, Y);
title("Q1.d Discrete Time Response of Y");
legend("Y(1)", "Y(2)")
figure(8)
stairs(Tsim, X);
title("Q1.d Discrete Time Response of X");
legend("X(1)", "X(2)", "X(3)", "X(4)")

figure(9)
plot(Tsim, X(:,1));
hold on

for t = [0.01 0.050 0.1]
    T = 0:t:10; 
    sysd = c2d(sys, t);
    X0 = [1, 1, 1, 2].';
    U = zeros(2, length(T));
    
    [Y, Tsim, X] = lsim(sysd, U, T, X0);
    
    stairs(Tsim, X(:,1))
end
legend("continuous", "0.01", "0.05", "0.1")
hold off
%% e 

clear

% Create System
A = [12.5314 , -91.36 , 28.7129 , 59.9628 ; 
     21.316 , -115.6631 , 37.5584 , 75.0519 ; 
     13.967 , -53.5817 , 15.4161 , 33.4391 ; 
     20.5222 , -123.3107 , 42.267 , 79.7156]; %4x4

B = [1.7649 , 1.7649; 
     2.8318 , 2.8318; 
     2.2353 , 2.2353; 
     2.7294 , 2.7294]; %2x4
 
C = [-0.46166 , -4.4635 , 1.9151 , 3.7274 ;
      1.0853 , -12.82 , 4.5753 , 8.3759]; %4x2
  
D = [0 , 0 ; 
     0 , 0 ]; %2x2

sys = ss(A,B,C,D);
timeStep = 0.5;
T = 0:timeStep:10; 
% End Create System
sysd = c2d(sys, timeStep);

X0 = [1, 1, 1, 2].';  % Column vector of initial state
U = zeros(size(T));  % Set up with no inputs 2x1001
U = repmat(U, 2, 1);

[Y, Tsim, X] = lsim(sysd, U, T, X0);
figure(9)
plot(Tsim, Y);
title("Q1.e Discrete T_S Changed Time Response of Y");
legend("Y(1)", "Y(2)")
figure(10)
plot(Tsim, X);
title("Q1.e Discrete T_S Changed Time Response of X");
legend("X(1)", "X(2)", "X(3)", "X(4)")
%% f

clear

% Create System
A = [12.5314 , -91.36 , 28.7129 , 59.9628 ; 
     21.316 , -115.6631 , 37.5584 , 75.0519 ; 
     13.967 , -53.5817 , 15.4161 , 33.4391 ; 
     20.5222 , -123.3107 , 42.267 , 79.7156]; %4x4

B = [1.7649 , 1.7649; 
     2.8318 , 2.8318; 
     2.2353 , 2.2353; 
     2.7294 , 2.7294]; %2x4
 
C = [-0.46166 , -4.4635 , 1.9151 , 3.7274 ;
      1.0853 , -12.82 , 4.5753 , 8.3759]; %4x2
  
D = [0 , 0 ; 
     0 , 0 ]; %2x2
% End Create System

T = 0:0.01:10;
T_idx = 2:1:length(T);
X0 = [1, 1, 1, 2].';  % Column vector of initial state
X = zeros(4, length(T));
X(:,1) = X0;

k_range = 10;
for t = T_idx
    phi = 0;
    for k = 0:1:k_range
       phi = phi + ((T(t)-T(t-1))*A)^k;
    end
    X(:,t) = expm((T(t) - T(t-1))*A)*X(:,(t-1));
end

figure(11)
stairs(T.',X.')
title("Q1.f Matrix Exponential time response of X")
legend("X(1)", "X(2)", "X(3)", "X(4)")
Y = C * X;
figure(12)
stairs(T.',Y.')
title("Q1.f Matrix Exponential time response of Y")
legend("Y(1)", "Y(2)")