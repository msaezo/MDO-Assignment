function [CL_w,CD_w] = Aerodynamics(sweep,b,lambda_in,lambda_out,root,epsilon_in,epsilon_out,CST,W_fuel,W_wing_c)
%matlab function for MDO assignment Aerodynamics
%Design variables: Sweep,b,lambda_in,lambda_out,root,epsilon_in,epsilon_out,CST
%%local design variable: x1
%MDA variables: W_fuel,W_wing
Res = Q3D_Start(sweep,b,lambda_in,lambda_out, root, epsilon_in,epsilon_out,CST,W_fuel,W_wing_c,0);
CL_w = Res.CLwing;
CD_w = Res.CDwing;

end