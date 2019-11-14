%%%_____Routine to write the input file for the EMWET procedure________% %%
function [] = write_init(sweep,b,lambda_in,lambda_out,root,epsilon_in,epsilon_out,W_fuel,W_wing)
W_AW        =    (29323-4280)*9.81;   %[N]
MTOW        =    W_fuel + W_wing + W_AW;    %[N]
MTOW        =    MTOW/9.81;     %[kg]
MZF         =    MTOW-W_fuel/9.81;         %[kg]
nz_max      =    2.5;   
span        =    b;            %[m]         
chord_kink  =    root*lambda_in;
tip         =    chord_kink*lambda_out;
Y_kink      =    5.616;
sweep_kink  =    5.37;
sweep_out   =    sweep;

X_kink      =    root-chord_kink+tand(sweep_kink)*Y_kink;
sweep_in    =    atand(X_kink/Y_kink);

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


 
spar_front  =    0.2;
spar_rear   =    0.6;
ftank_start =    0.1;
ftank_end   =    0.85;
eng_num     =    0;
E_al        =    7.1E10;       %N/m2
rho_al      =    2800;         %kg/m3
Ft_al       =    2.95E8;        %N/m2
Fc_al       =    2.95E8;        %N/m2
pitch_rib   =    0.5;          %[m]
eff_factor  =    0.96;         %Depend on the stringer type
Airfoil_root=    'Airfoil_root';
Airfoil_tip =    'Airfoil_tip';

section_num =    3;
airfoil_num =    2;
wing_surf   =    S;

fid = fopen( 'Fokker70test.init','wt');
fprintf(fid, '%g %g \n',MTOW,MZF);
fprintf(fid, '%g \n',nz_max);

fprintf(fid, '%g %g %g %g \n',wing_surf,span,section_num,airfoil_num);

fprintf(fid, '0 %s \n',Airfoil_root);
fprintf(fid, '1 %s \n',Airfoil_tip);
fprintf(fid, '%g %g %g %g %g %g \n',AC.Wing.Geom(1,4),AC.Wing.Geom(1,1),AC.Wing.Geom(1,2),AC.Wing.Geom(1,3),spar_front,spar_rear);
fprintf(fid, '%g %g %g %g %g %g \n',AC.Wing.Geom(2,4),AC.Wing.Geom(2,1),AC.Wing.Geom(2,2),AC.Wing.Geom(2,3),spar_front,spar_rear);
fprintf(fid, '%g %g %g %g %g %g \n',AC.Wing.Geom(3,4),AC.Wing.Geom(3,1),AC.Wing.Geom(3,2),AC.Wing.Geom(3,3),spar_front,spar_rear);

fprintf(fid, '%g %g \n',ftank_start,ftank_end);

fprintf(fid, '%g \n', eng_num);

fprintf(fid, '%g %g %g %g \n',E_al,rho_al,Ft_al,Fc_al);
fprintf(fid, '%g %g %g %g \n',E_al,rho_al,Ft_al,Fc_al);
fprintf(fid, '%g %g %g %g \n',E_al,rho_al,Ft_al,Fc_al);
fprintf(fid, '%g %g %g %g \n',E_al,rho_al,Ft_al,Fc_al);

fprintf(fid,'%g %g \n',eff_factor,pitch_rib);
fprintf(fid,'0 \n');
fclose(fid);
end