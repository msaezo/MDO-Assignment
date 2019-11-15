function [c,ceq] = constraints(x)
%function computing constraint on the wing problem
%NB constraints for MDF are directly computed from the values of the
%coupling variables. To pass those on, use a global variable. Otherwise,
%matlab will need to re-run the MDA and disciplinary analyses.

%Variables definition:
rho_fuel = 0.81715E3; %Density of fuel
f_tank = 0.93; %Usable space

%Variables initial vector
S_0 = 93.5;
MTOW_0 = (29323+5000)*9.81;


global couplings;

W_fuel = couplings.W_fuel;
W_wing = couplings.W_wing;

W_AW = (29323-4280)*9.81;
MTOW = W_fuel + W_wing + W_AW;

V_tank = VolumeTank(x)*f_tank;
V_fuel = W_fuel/rho_fuel;

S = x(4)/2*(x(2)*(x(3)+1)); %Surface area calculation

c1 = -1;
% c1 = (V_fuel - V_tank)/V_fuel;
c2 = (MTOW/S-MTOW_0/S_0)/(MTOW_0/S_0);

c = [c1,c2];
ceq = [];
end