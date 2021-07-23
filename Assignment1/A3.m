close all
G = tf(1000, [1 1000]);
ts = 0.0001;




Z_BiLinear = c2d(G, ts, 'tustin');

opt = c2dOptions('Method','tustin','PrewarpFrequency',3.4);
Z_BiLinearPreWarp = c2d(G, ts, opt);

Z_Matched= c2d(G, ts, 'matched');

Z_ImpulseInvariant = c2d(G, ts, 'impulse');

Z_ZOH = c2d(G, ts);

systems = {"Original" G;
           "Tustin" Z_BiLinear;
           "Prewarped Tustin" Z_BiLinearPreWarp;
           "Matched" Z_Matched;
           "Impulse Invariant" Z_ImpulseInvariant
           "Zero Order Hold" Z_ZOH};

for i = 1:1:length(systems)
    figure(i)
    subplot(2,2,[1 2])
    bode(systems{i,2})
    subplot(2,2,3)
    step(systems{i,2})
    subplot(2,2,4)
    impulse(systems{i,2})
    sgtitle(systems{i,1}) 
end