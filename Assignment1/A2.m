G = zpk([], -2, 4);
G.IODelay = 0.2;
figure(1)
nyquist(2.3 * G);