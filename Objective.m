function [MTOW_new_norm] = Objective(W_fuel,W_wing)
%matlab function for MDO assignment objective
%Design variables: --
%%local design variable: W_AW
%MDA variables: W_fuel,W_wing

MTOW_0 = (29323+5000)*9.81;

W_AW = (29323-4280)*9.81;

MTOW_new = W_fuel + W_AW + W_wing;
MTOW_new_norm = MTOW_new/MTOW_0;

end