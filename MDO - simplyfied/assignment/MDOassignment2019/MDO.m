clc
clear
S = 93.50;
b = 28.08;
Y_kink = 0.4*0.5*28.08;
taper = 0.235;
MAC = 3.80;
AR = 8.43;
Cr = 2*S/(b*(taper+1));
% MAC = 2/3*Cr*((1+taper+taper^2)/(1+taper));
sweep_c4 = 17.45;
sweep_LE = atand(tand(sweep_c4)+0.25*(2*Cr/b)*(1-taper));
sweep_TE = atand(tand(sweep_LE)-(2*Cr/b)*(1-taper));