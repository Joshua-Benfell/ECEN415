clear
close all
f = 180713;  % Unity Gain Bandwidth
f_s = 10 * f;  % Sampling Frequency 10 times Unity Gain Bandwidth
% Need to attenuate frequencies greater than f_s / 2 by - 40
% So need to have a corner frequency of a decade less
f_c = f_s / (2 * 10);
w_c = 2*pi * f_c;

s = tf('s');

G_S = (1 - exp(-s/f_s))/s;
G_B =tf(w_c^2, [1 sqrt(2)*w_c w_c^2]);
title("Sampler")
title("Anti-Aliasing")
[MAG, PHASE] = bode(1.8074e+06 * G_S * G_B, 10000 * 2*pi)
bode(1.8074e+06 * G_S * G_B);
title("Cascaded")