clear;

figure(1);
G_1 = tf([20 20 10], [1 11 10 0]);
%step(feedback(G_1,1));
nyquist(G_1);

figure(2);
G_2 = tf([20 20 10], [1 9 -10 0]);
%step(feedback(G_2,1))
nyquist(G_2);


figure(3);
G_3 = tf([1 0 3], [1 2 1]);
nyquist(G_3);

figure(4);
G_4 = tf([3 3], [1 -10 0]);
nyquist(G_4);
%step(feedback(1000000*G_4,1))