%script to test function D_airfoil2

clear;
close all;
clc;


Au = [0.2252    0.0904    0.2445    0.1314    0.4253];    %upper-surface Bernstein coefficients
Al = [-0.2413   -0.0557   -0.3071  -0.2166    0.4077];    %lower surface Bernstein coefficients

X = linspace(0,1,99)';      %points for evaluation along x-axis

[Xtu,Xtl,C] = D_airfoil2(Au,Al,X);



hold on
plot(Xtu(:,1),Xtu(:,2),'b');    %plot upper surface coords
plot(Xtl(:,1),Xtl(:,2),'b');    %plot lower surface coords
%plot(X,C,'r');                  %plot class function
ylim([-0.5 0.5]);


