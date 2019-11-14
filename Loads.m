function [] = Loads(sweep,b,lambda_in,lambda_out,root,epsilon_in,epsilon_out,CST,W_fuel,W_wing)
%matlab function for MDO assignment Loads
%Design variables: Sweep,b,lambda_in,lambda_out,root,epsilon_in,epsilon_out,CST
%%local design variable: x1
%MDA variables: MTOW

[Res,q,MAC] = Q3D_Start_inviscid(sweep,b,lambda_in,lambda_out, root, epsilon_in,epsilon_out, CST,W_fuel,W_wing,0);
x = linspace(0,b/2,20);
Y_span = Res.Wing.Yst;
L_span = Res.Wing.cl.*Res.Wing.chord*q;
L_poly_coeff = polyfit(Y_span,L_span,7);
L_span = L_poly_coeff(1)*x.^7+L_poly_coeff(2)*x.^6+L_poly_coeff(3)*x.^5+L_poly_coeff(4)*x.^4+L_poly_coeff(5)*x.^3+L_poly_coeff(6)*x.^2+L_poly_coeff(7)*x.^1+L_poly_coeff(8);

M_span = Res.Wing.cm_c4.*Res.Wing.chord*q*MAC;
M_poly_coeff = polyfit(Y_span,M_span,7);
M_span = M_poly_coeff(1)*x.^7+M_poly_coeff(2)*x.^6+M_poly_coeff(3)*x.^5+M_poly_coeff(4)*x.^4+M_poly_coeff(5)*x.^3+M_poly_coeff(6)*x.^2+M_poly_coeff(7)*x.^1+M_poly_coeff(8);

x = x/(b/2);

fid = fopen( 'Fokker70test.load','wt');
fprintf(fid, '%g %g %g \n',x(1),L_span(1),M_span(1));
fprintf(fid, '%g %g %g \n',x(2),L_span(2),M_span(2));
fprintf(fid, '%g %g %g \n',x(3),L_span(3),M_span(3));
fprintf(fid, '%g %g %g \n',x(4),L_span(4),M_span(4));
fprintf(fid, '%g %g %g \n',x(5),L_span(5),M_span(5));
fprintf(fid, '%g %g %g \n',x(6),L_span(6),M_span(6));
fprintf(fid, '%g %g %g \n',x(7),L_span(7),M_span(7));
fprintf(fid, '%g %g %g \n',x(8),L_span(8),M_span(8));
fprintf(fid, '%g %g %g \n',x(9),L_span(9),M_span(9));
fprintf(fid, '%g %g %g \n',x(10),L_span(10),M_span(10));
fprintf(fid, '%g %g %g \n',x(11),L_span(11),M_span(11));
fprintf(fid, '%g %g %g \n',x(12),L_span(12),M_span(12));
fprintf(fid, '%g %g %g \n',x(13),L_span(13),M_span(13));
fprintf(fid, '%g %g %g \n',x(14),L_span(14),M_span(14));
fprintf(fid, '%g %g %g \n',x(15),L_span(15),M_span(15));
fprintf(fid, '%g %g %g \n',x(16),L_span(16),M_span(16));
fprintf(fid, '%g %g %g \n',x(17),L_span(17),M_span(17));
fprintf(fid, '%g %g %g \n',x(18),L_span(18),M_span(18));
fprintf(fid, '%g %g %g \n',x(19),L_span(19),M_span(19));
fprintf(fid, '%g %g %g \n',x(20),L_span(20),M_span(20));

fclose(fid);
end