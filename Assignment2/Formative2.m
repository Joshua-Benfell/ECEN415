%% a
clear

R = 1000;
C = 1e-3;

R1 = 1*R;
R2 = R1*56;

A = [
     -2/(R*C), 1/(R*C), -R2/(R1 * R * C);
     1/(R*C), -2/(R*C), 1/(R*C);
     0, 1/(R*C), (-1/(R*C))-(1/(R1*C))
    ];

pts = 10000;
T = linspace(0,10,pts);
T_idx = 2:1:pts;

X0 = [1, 0, 0].';
X = zeros(size(T));
X = repmat(X, 3, 1);
X(:,1) = X0;


for t = T_idx
    X(:,t) = expm((T(t) - T(t-1))*A)*X(:,(t-1));
end

plot(T,X)
legend("V_a", "V_b", "V_c");
title("Initial State = [1 0 0]")

%% c
[eigenvectors, eigenvalues] = eig(A);

for idx = [1 2 3]
    X0 = eigenvectors(idx,:).';
    X = zeros(3, length(T));
    X(:,1) = X0;
    for t = T_idx
        X(:,t) = expm((T(t) - T(t-1))*eigenvalues(idx,idx))*X(:,(t-1));
    end
    
    figure(idx + 1);
    plot(T,X)
    legend("V_a", "V_b", "V_c");
    title("\lambda = " + strjoin(string(eigenvalues(idx,idx))) + " | \nu = " + strjoin(string(X0)))
end

