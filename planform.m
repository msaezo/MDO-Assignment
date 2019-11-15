function[S]=planform(sweep,b,lambda_in,lambda_out, root, Plotting)
%% Aerodynamic solver setting for inviscid calculation

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
AC.Wing.Geom(3,1) = Y_kink*tand(sweep_in)+(b/2-Y_kink)*tand(sweep_out);
AC.Wing.Geom(3,2) = b/2;
AC.Wing.Geom(3,3) = 0;
AC.Wing.Geom(3,4) = tip;

S_in = (AC.Wing.Geom(1,4)+AC.Wing.Geom(2,4))*AC.Wing.Geom(2,2)*0.5;                      %inboard surface
S_out = (AC.Wing.Geom(3,4)+AC.Wing.Geom(2,4))*(AC.Wing.Geom(3,2)-AC.Wing.Geom(2,2))*0.5;  %inboard surface
S = 2*(S_in+S_out);                     %total surfaceMAC
lambda_in = AC.Wing.Geom(2,4)/AC.Wing.Geom(1,4);
lambda_out = AC.Wing.Geom(3,4)/AC.Wing.Geom(2,4);
MAC_in = 2/3*AC.Wing.Geom(1,4)*((1+lambda_in+lambda_in^2)/(1+lambda_in));
MAC_out = 2/3*AC.Wing.Geom(2,4)*((1+lambda_out+lambda_out^2)/(1+lambda_out));
MAC = (MAC_in*S_in+MAC_out*S_out)/(S_in+S_out);


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

%     plot([AC.Wing.Geom(1,1) AC.Wing.Geom(1,4)], -[AC.Wing.Geom(1,2) AC.Wing.Geom(1,2)])
%     plot([AC.Wing.Geom(2,1) AC.Wing.Geom(2,1)+AC.Wing.Geom(2,4)], -[AC.Wing.Geom(2,2) AC.Wing.Geom(2,2)])
%     plot([AC.Wing.Geom(3,1) AC.Wing.Geom(3,1)+AC.Wing.Geom(3,4)], -[AC.Wing.Geom(3,2) AC.Wing.Geom(3,2)])
%     plot([AC.Wing.Geom(1,1) AC.Wing.Geom(2,1)], -[AC.Wing.Geom(1,2) AC.Wing.Geom(2,2)])
%     plot([AC.Wing.Geom(2,1) AC.Wing.Geom(3,1)], -[AC.Wing.Geom(2,2) AC.Wing.Geom(3,2)])
%     plot([AC.Wing.Geom(1,1)+AC.Wing.Geom(1,4) AC.Wing.Geom(2,1)+AC.Wing.Geom(2,4)], -[AC.Wing.Geom(1,2) AC.Wing.Geom(2,2)])
%     plot([AC.Wing.Geom(2,1)+AC.Wing.Geom(2,4) AC.Wing.Geom(3,1)+AC.Wing.Geom(3,4)], -[AC.Wing.Geom(2,2) AC.Wing.Geom(3,2)])
    hold off
else
end

end