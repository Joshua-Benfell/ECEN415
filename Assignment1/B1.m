%a
close all
D = tf(1);
delay = 0.1;
D.IODelay = delay;
figure(1)
bode(D);