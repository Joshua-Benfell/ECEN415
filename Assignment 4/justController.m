function[u1, u2] = controller_command(t, x)
    global eta x_0 g M c_d J L k
    if (length(x) == 7)
       x(8:9) = [1000 0.1];
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
    % Linearise at point t.
    df = jacobian(f_sym, [x1 x2 x3 x4 x5 x6 x7]);
    dg = jacobian(f_sym, [i1 i2]);
    A = double(subs(df, [x1 x2 x3 x4 x5 x6 x7 i1 i2], x));
    B = double(subs(dg, [x1 x2 x3 x4 x5 x6 x7 i1 i2], x));    
    %x_f = [1500, 130, 100, 20, 10, 0, 0];
    x_curr = x(1:7).';
    % Open loop straight up for 4 seconds to help us gain the height.
    if t <4
        x_desired = [100000*(t+1)/15;
                     130; 
                     0; 
                     0; 
                     0;
                     0; 
                     1000*(1-(t)/15)
                     ];
        u = pinv(B) * (x_desired-x_curr-A*x_curr)
        
        if (u(1) < 0)
            u(1) = 0; 
        end
        if (u(2) < 0)
            u(2) = 0; 
        end
        %u2 = u(2);
        %u1 = u(1);
        u1 = 150000;
        u2 = 0;
    % LQR to reach the set points I guess.
    else
        % zero prevention because infinity doesn't play nice
        if x(8) == 0
            x(8) = 0.1;
        end
        if x(9) == 0
            x(9) = 0.1;
        end
        u_curr = x(8:9).'
        
        % Not really the set point because I have no idea how this works.
        % But this does.
        look_ahead = 4;
        x_desired = [min(1500, 1500*(t+1)/15);
                     min(100, 100*(1-(t+1)/15)+0.1); 
                     min(50, 50*(t+look_ahead)/15); 
                     min(20, 20*(t+look_ahead)/15); 
                     10;
                     5*(1-(t+1)/15)+0.1;
                     1000*(1-(t+1)/15)+0.1
                     ];
        % Tolerances we want on the each state variable         
        x_bounds = [1000; 50; 0.1; 1; 1; 0.01; 1000];
        % Input tolerances
        u_bounds = [1 1].';

        % Make Q and R Matrices
        Q = diag((x_bounds.^2)./x_desired.^2)
        R = diag((u_bounds.^2)./u_curr.^2)
        % dlqr because we're sampling every one second by the rocket works
        % in much smaller timesteps
        [K,P,e] = lqr(A, B, Q, R)
        % Calc input and scale because for some reason the scale became
        % basically zero0
        u = -K*(x_curr-x_desired)
        % Prevent negative side booster because we don't have time for
        % that.
        % Limit magnitude of the inputs to prevent fuel from being used up
        % too quickly.
        u1 = max(min(1.5*10^5,u(1)),0);
        
        u2Bound = 4*10^3;
        if u(2) > u2Bound
            u2 = u2Bound;
        elseif u(2) < -u2Bound
            u2 = -u2Bound;
        else
            u2 = u(2);
        end
        %Wonderful angle slight fix
        %if t >= 13
        %    u2 = -2500
        %end
    end

    
    %u1 = eta*x_0(7)/15; % Burn all of the fuel in 15s at uniform rate. 
    %u2 = eta*2;
end