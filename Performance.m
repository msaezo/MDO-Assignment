function [W_fuel_new] = Performance(W_fuel,W_wing,CL_w,CD_w)
%matlab function for MDO assignment Performance
%Design variables: -
%%local design variable: x1
%MDA variables: W_fuel,W_wing,CL_w,CD_w

%Variables
R = 2000*10^3; %This is the design Range (http://www.flyfokker.com/sites/default/files/FLYFokker/FlyFokker_PDF_Fokker70_Performance.pdf)
V = 222.222; %This is the long range cruise speed
Ct = 1.8639E-4;
W_AW = (29323-4280)*9.81;
CD_AW = 0.0093; %Value for aircraft minus wing CD

CD_CL = (CD_w+CD_AW)/CL_w;
Wst_end = exp(R*Ct/V*(CD_CL));
MTOW = W_AW + W_wing +W_fuel;
W_fuel_new = (1-0.938*(1/Wst_end))*MTOW;


end