clear
close all
figure(1)
subplot(1,2,1)
G = tf(15, [1 3 2])
rlocus(G)
title("G(s)")
subplot(1,2,2)
s = tf('s');
ts = -100000;
sample = (1 - pade(exp(ts*s),1))/s
rlocus(G*sample)
title("G(s) Sampled")
figure(2)
title("G(s)")
margin(G)
figure(3)
ts = -0.176999;
%ts = -0.000001
sample = (1 - pade(exp(ts*s),1))/s;
rlocus(G * sample)
figure(4)
title("G(s) Sampled")
margin(G * sample)