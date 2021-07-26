%a
close all
D = tf(1);
td = 0.1;
D.IODelay = td;
figure(1)
bode(D);

figure(2)
D = tf(1)
delays = 0.00025:0.00001:0.0003;
for td = delays
    D.IODelay = td;
    hold on
    margin(pade(D, 2))
end
delays = delays * 1000000;
legendCell = cellstr(num2str(delays', 'Delay (us) = %.1f'));
legend(legendCell)