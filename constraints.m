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
MTOW_0 = (29323+6000)*9.81;

% Wing planform geometry
global couplings;
X0 = couplings.X0;
W_fuel = couplings.W_fuel;
W_wing = couplings.W_wing;

x = x.*abs(X0);
sweep = x(1);
b = x(2);
lambda_in = x(3);
lambda_out = x(4);
root = x(5);
epsilon_in = x(6);
epsilon_out = x(7);
CST = [x(8),x(9),x(10),x(11),x(12),x(13),x(14),x(15),...
        x(16),x(17),x(18),x(19),x(20),x(21),x(22),x(23),x(24),x(25),...
        x(26),x(27),x(28),x(29),x(30),x(31)];
Au_root = [x(8),x(9),x(10),x(11),x(12),x(13)];
Al_root = [x(14),x(15),x(16),x(17),x(18),x(19)];
    
chord_kink = root*lambda_in;
tip = chord_kink*lambda_out;
Y_kink = 5.616;
sweep_kink = 5.37;

sweep_out = sweep;
X_kink = root-chord_kink+tand(sweep_kink)*Y_kink;
sweep_in  = atand(X_kink/Y_kink);

AC.Wing.Geom = [];
AC.Wing.Geom(1,1) = 0;
AC.Wing.Geom(1,2) = 0;
AC.Wing.Geom(1,3) = 0;
AC.Wing.Geom(1,4) = root;
AC.Wing.Geom(1,5) = 0;
AC.Wing.Geom(2,1) = X_kink;
AC.Wing.Geom(2,2) = Y_kink;
AC.Wing.Geom(2,3) = 0;
AC.Wing.Geom(2,4) = root+Y_kink*tand(sweep_kink)-(Y_kink*tand(sweep));
AC.Wing.Geom(2,4) = chord_kink;
AC.Wing.Geom(2,5) = epsilon_in;
AC.Wing.Geom(3,1) = Y_kink*tand(sweep_in)+(b/2-Y_kink)*tand(sweep_out);
AC.Wing.Geom(3,2) = b/2;
AC.Wing.Geom(3,3) = 0;
AC.Wing.Geom(3,4) = tip;
AC.Wing.Geom(3,5) = epsilon_out;

S_in = (AC.Wing.Geom(1,4)+AC.Wing.Geom(2,4))*AC.Wing.Geom(2,2)*0.5;                      %inboard surface
S_out = (AC.Wing.Geom(3,4)+AC.Wing.Geom(2,4))*(AC.Wing.Geom(3,2)-AC.Wing.Geom(2,2))*0.5;  %inboard surface
S = 2*(S_in+S_out);                     %total surfaceMAC


W_AW = (29323-4280)*9.81;
MTOW = W_fuel + W_wing + W_AW;

V_tank = VolumeTank(x)*f_tank;
V_fuel = W_fuel/rho_fuel;


c1 = V_fuel/V_tank-1;
c2 = (MTOW/S)/(MTOW_0/S_0)-1;

c = [c1,c2];
ceq = [];
end