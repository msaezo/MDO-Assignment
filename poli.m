clc

Y_span  = [0.4667 1.4 2.3333 3.2667 4.2 5.1333 6.125 7.175 8.225 9.275 10.325 11.375 12.425 13.475];

L_span = [36923.5 37122.8 36716.1 35889.4 34735.3 33359.6 31745.6 30031.4 28181.4 26154.5 23896.7 21294.4 18038.3 13102.9];

L_poly_coeff = polyfit(Y_span,L_span,7);

x = linspace(0,b/2,20);
L = L_poly_coeff(1)*x.^7+L_poly_coeff(2)*x.^6+L_poly_coeff(3)*x.^5+L_poly_coeff(4)*x.^4+L_poly_coeff(5)*x.^3+L_poly_coeff(6)*x.^2+L_poly_coeff(7)*x.^1+L_poly_coeff(8);
% hold on
% plot(Y_span,L_span);
% plot(x,L);
% hold off