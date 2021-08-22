%% a
clear
% 200Hz
R = 32487;
C = 10*10^(-9);

R1 = 1*R;
R2 = R1*56;

A = [
     -2/(R*C), 1/(R*C), -R2/(R1 * R * C);
     1/(R*C), -2/(R*C), 1/(R*C);
     0, 1/(R*C), (-1/(R*C))-(1/(R1*C))
    ];

sys = ss(A);

pts = 10000;
T = linspace(0,0.01,pts);
T_idx = 2:1:pts;

X0 = [1, 0, 0].';
X = zeros(size(T));
X = repmat(X, 3, 1);
X(:,1) = X0;


for t = T_idx
    X(:,t) = expm((T(t) - T(t-1))*A)*X(:,(t-1));
end

plot(T,X)
