function [MTOW_new] = Objective(W_fuel,W_wing)
%matlab function for MDO assignment objective
%Design variables: --
%%local design variable: W_AW
%MDA variables: W_fuel,W_wing

MTOW_0 = (29323+5000)*9.81;

W_AW = (29323-4280)*9.81;

MTOW_new = W_fuel + W_AW + W_wing;
MTOW_new= MTOW_new/MTOW_0;

global couplings
couplings.MTOW_c = MTOW_new;
couplings.W_fuel_c = W_fuel;
end