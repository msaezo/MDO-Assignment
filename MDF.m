clc
clear

%Matlab MDF Gauss Seidel implementation of the Wing problem 
addpath('Q3D')
addpath('Storage')
addpath('matlab-jsystem-master')
%Initial values:
sweep = 24;
b = 28.08;
lambda_in = 0.6;
lambda_out = 0.4;
root = 5.8;
epsilon_in = 1;
epsilon_out = 1;
CST = [0.2337, 0.0796, 0.2683, 0.0887, 0.2789, 0.3811, -0.2254, -0.1634, -0.0470, -0.4771, 0.0735, 0.3255, 0.1385, 0.0472, 0.1590, 0.0526, 0.1653, 0.2258, -0.1336, -0.0968, -0.0279, -0.2827, 0.0435, 0.1929];
X0 = [sweep, b, lambda_in, lambda_out, root, epsilon_in, epsilon_out, CST];
x0  = X0./X0; 

%%only for plot
Au_root = [X0(8),X0(9),X0(10),X0(11),X0(12),X0(13)];
Al_root = [X0(14),X0(15),X0(16),X0(17),X0(18),X0(19)];

Au_tip = [X0(20),X0(21),X0(22),X0(23),X0(24),X0(25)];
Al_tip = [X0(26),X0(27),X0(28),X0(29),X0(30),X0(31)];

[X0tu_root,X0tl_root] = D_airfoil2(Au_root,Al_root);
[X0tu_tip,X0tl_tip] = D_airfoil2(Au_tip,Al_tip);
global airfoil;
airfoil.X0tu_root = X0tu_root;
airfoil.X0tl_root = X0tl_root;
airfoil.X0tu_tip = X0tu_tip;
airfoil.X0tl_tip = X0tl_tip;

%bounds

lb = [20,24.08,0.4,0.35,4.4,-4,-4,CST(1)-0.02,CST(2)-0.02,CST(3)-0.02,CST(4)-0.02,...
    CST(5)-0.02,CST(6)-0.02,CST(7)-0.02,CST(8)-0.02,CST(9)-0.02,CST(10)-0.02,CST(11)-0.02...
    ,CST(12)-0.02,CST(13)-0.02,CST(14)-0.02,CST(15)-0.02,CST(16)-0.02,...
    CST(17)-0.02,CST(18)-0.02,CST(19)-0.02,CST(20)-0.02,CST(21)-0.02,CST(22)-0.02,CST(23)-0.02...
    ,CST(24)-0.02]./X0;
ub = [30,35.08,0.8,0.8,6.4,4,4,CST(1)+0.02,CST(2)+0.02,CST(3)+0.02,CST(4)+0.02,...
    CST(5)+0.02,CST(6)+0.02,CST(7)+0.02,CST(8)+0.02,CST(9)+0.02,CST(10)+0.02,CST(11)+0.02...
    ,CST(12)+0.02,CST(13)+0.02,CST(14)+0.02,CST(15)+0.02,CST(16)+0.02,...
    CST(17)+0.02,CST(18)+0.02,CST(19)+0.02,CST(20)+0.02,CST(21)+0.02,CST(22)+0.02,CST(23)+0.02...
    ,CST(24)+0.02]./X0;

global couplings;
couplings.X0 = X0;
couplings.W_wing_c = 4280*9.81;
couplings.W_fuel_c = 5000*9.81;
% Options for the optimization
options.Display         = 'iter-detailed';
options.Algorithm       = 'sqp';
options.FunValCheck     = 'off';
options.DiffMinChange   = 1e-6;         % Minimum change while gradient searching
options.DiffMaxChange   = 1e-3;         % Maximum change while gradient searching
options.TolCon          = 1e-6;         % Maximum difference between two subsequent constraint vectors [c and ceq]
options.TolFun          = 1e-6;         % Maximum difference between two subsequent objective value
options.TolX            = 1e-6;         % Maximum difference between two subsequent design vectors

options.MaxIter         = 30;           % Maximum iterations


tic;
[x,FVAL,EXITFLAG,OUTPUT] = fmincon(@(x) Optim_MDFGauss(x),x0,[],[],[],[],lb,ub,@(x) constraints(x),options);
toc;
load handel
sound(y,Fs)
%[f,vararg] = Optim_MDFGauss(x);
