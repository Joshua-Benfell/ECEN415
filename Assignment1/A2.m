close all
%a
G = zpk([], -2, 4);
G.IODelay = 0.2;
figure(1)
nyquist(G);
figure(2)
nyquist(2.298850575*G);
%b
figure(3);
step(feedback(G, 1));
figure(4);
step(feedback(2.298* G, 1));
%c
close all 
G = zpk([], -2, 4);
figure(1)
delays = 0.1:0.1:0.7;
for i = delays
    G.IODelay = i;
    nyquist(G);
    hold on
end
legendCell = cellstr(num2str(delays', 'Delay = %.1f'));
legend(legendCell)

