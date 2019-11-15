function[Res]=Q3D_Start(sweep,b,lambda_in,lambda_out, root, epsilon_in,epsilon_out, CST, W_fuel,W_wing_c,Plotting)
%% Aerodynamic solver setting for viscid calculation

% Wing planform geometry
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
lambda_in = AC.Wing.Geom(2,4)/AC.Wing.Geom(1,4);
lambda_out = AC.Wing.Geom(3,4)/AC.Wing.Geom(2,4);
MAC_in = 2/3*AC.Wing.Geom(1,4)*((1+lambda_in+lambda_in^2)/(1+lambda_in));
MAC_out = 2/3*AC.Wing.Geom(2,4)*((1+lambda_out+lambda_out^2)/(1+lambda_out));
MAC = (MAC_in*S_in+MAC_out*S_out)/(S_in+S_out);

% Wing incidence angle (degree)
AC.Wing.inc  = 0;   
            
% Airfoil coefficients input matrix
%                    | ->     upper curve coeff.                <-|   | ->       lower curve coeff.       <-| 
AC.Wing.Airfoils   = [CST(1:12);CST(13:24)];
AC.Wing.eta = [0;1];  % Spanwise location of the airfoil sections

% Viscous vs inviscid
AC.Visc  = 1;              % 0 for inviscid and 1 for viscous analysis
AC.Aero.MaxIterIndex = 150;    %Maximum number of Iteration for the
                                %convergence of viscous calculation
                                
% Aircraft Design Parameters
W_AW = (29323-4280)*9.81;
MTOW = W_fuel + W_wing_c + W_AW;                                
% Flight Condition

AC.Aero.rho   = 0.422620;                         % air density  (kg/m3)
AC.Aero.alt   = 9807.8;                           % flight altitude (m)
AC.Aero.M     = 0.74;                             % flight Mach number 
speed_of_sound  = 308.381;                        % at 7924.8m
AC.Aero.V     = AC.Aero.M*speed_of_sound;         % flight speed (m/s)
d_visc = 0.0000147581;                 %at 7924.8m
AC.Aero.Re    = AC.Aero.rho*MAC*AC.Aero.V/d_visc;        % reynolds number (bqased on mean aerodynamic chord)
n = 1;
q = 0.5*AC.Aero.rho*AC.Aero.V^2;
MTOW_des = sqrt(MTOW*(MTOW-W_fuel));
AC.Aero.CL    = MTOW_des*n/(q*S);          % lift coefficient - comment this line to run the code for given alpha%
% AC.Aero.Alpha = 2;             % angle of attack -  comment this line to run the code for given cl 


%% 
tic

Res = Q3D_solver(AC);
toc



%% Plot Geometry
if Plotting == 1
    hold on
    axis equal
    plot([AC.Wing.Geom(1,1) AC.Wing.Geom(1,4)], [AC.Wing.Geom(1,2) AC.Wing.Geom(1,2)])
    plot([AC.Wing.Geom(2,1) AC.Wing.Geom(2,1)+AC.Wing.Geom(2,4)], [AC.Wing.Geom(2,2) AC.Wing.Geom(2,2)])
    plot([AC.Wing.Geom(3,1) AC.Wing.Geom(3,1)+AC.Wing.Geom(3,4)], [AC.Wing.Geom(3,2) AC.Wing.Geom(3,2)])
    plot([AC.Wing.Geom(1,1) AC.Wing.Geom(2,1)], [AC.Wing.Geom(1,2) AC.Wing.Geom(2,2)])
    plot([AC.Wing.Geom(2,1) AC.Wing.Geom(3,1)], [AC.Wing.Geom(2,2) AC.Wing.Geom(3,2)])
    plot([AC.Wing.Geom(1,1)+AC.Wing.Geom(1,4) AC.Wing.Geom(2,1)+AC.Wing.Geom(2,4)], [AC.Wing.Geom(1,2) AC.Wing.Geom(2,2)])
    plot([AC.Wing.Geom(2,1)+AC.Wing.Geom(2,4) AC.Wing.Geom(3,1)+AC.Wing.Geom(3,4)], [AC.Wing.Geom(2,2) AC.Wing.Geom(3,2)])

    plot([AC.Wing.Geom(1,1) AC.Wing.Geom(1,4)], -[AC.Wing.Geom(1,2) AC.Wing.Geom(1,2)])
    plot([AC.Wing.Geom(2,1) AC.Wing.Geom(2,1)+AC.Wing.Geom(2,4)], -[AC.Wing.Geom(2,2) AC.Wing.Geom(2,2)])
    plot([AC.Wing.Geom(3,1) AC.Wing.Geom(3,1)+AC.Wing.Geom(3,4)], -[AC.Wing.Geom(3,2) AC.Wing.Geom(3,2)])
    plot([AC.Wing.Geom(1,1) AC.Wing.Geom(2,1)], -[AC.Wing.Geom(1,2) AC.Wing.Geom(2,2)])
    plot([AC.Wing.Geom(2,1) AC.Wing.Geom(3,1)], -[AC.Wing.Geom(2,2) AC.Wing.Geom(3,2)])
    plot([AC.Wing.Geom(1,1)+AC.Wing.Geom(1,4) AC.Wing.Geom(2,1)+AC.Wing.Geom(2,4)], -[AC.Wing.Geom(1,2) AC.Wing.Geom(2,2)])
    plot([AC.Wing.Geom(2,1)+AC.Wing.Geom(2,4) AC.Wing.Geom(3,1)+AC.Wing.Geom(3,4)], -[AC.Wing.Geom(2,2) AC.Wing.Geom(3,2)])
    hold off
else
end

end