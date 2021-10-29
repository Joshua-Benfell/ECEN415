function[u1, u2] = controller_command(t, x)
    global eta x_0 g M c_d J L k
    if (length(x) == 7)
       x(8:9) = [100000 10000];
    end
    if x(8) == 0
        x(8) = 100000;
    end

    syms x1 x2 x3 x4 x5 x6 x7 i1 i2
    f_sym = @(t,x) [x2; 
                    g + i1*cos(x5) / (M+x7) - c_d*x2^2*exp(-x1/10400);  
                    x4;
                    i1*sin(x5) / (M+x7);
                    x6;
                    k/(J+L^2*x7)*i2;
                    -1/eta*(abs(i1)+abs(i2));
                    ];
    df = jacobian(f_sym, [x1 x2 x3 x4 x5 x6 x7]);
    dg = jacobian(f_sym, [i1 i2]);
    A = double(subs(df, [x1 x2 x3 x4 x5 x6 x7 i1 i2], x));
    B = double(subs(dg, [x1 x2 x3 x4 x5 x6 x7 i1 i2], x));    
    %x_f = [1500, 130, 100, 20, 10, 0, 0];
    x_desired = [1000000*(t+1)/15;
                 250*(t+1)/15; 
                 10; 
                 5; 
                 10;
                 1*(1-(t)/15); 
                 100*(1-(t)/15)
                 ];
        
    x_curr = x(1:7).';
    u_curr = x(8:9).';
    
    x_bounds = [100; 50; 25; 1; 0.5; 0.1; 0.1];
    u_bounds = [10000000 100000000].';
    
    %u = pinv(B) * (x_desired-x_curr-A*x_curr)
    Q = diag((x_bounds.^2)./x_desired.^2)
    R = diag((u_bounds)./u_curr.^2)
    [K,P,e] = lqr(A, B, Q, R)
    u = -K*x_curr
    if (u(1) < 0)
       u(1) = 0; 
    end
    if (u(2) < 0)
       u(2) = 0.1; 
    end
    u1 = min(3*10^5,u(1));
    %u1 = u(1);
    u2 = u(2);

    
    %u1 = eta*x_0(7)/15; % Burn all of the fuel in 15s at uniform rate. 
    %u2 = eta*2;
end