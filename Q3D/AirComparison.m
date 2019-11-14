M = 6;  %Number of CST-coefficients in design-vector x

%Define optimization parameters
x0 = 0.5*ones(M,1);     %initial value of design vector x(starting vector for search process)
lb = -1*ones(M,1);    %upper bound vector of x
ub = 1*ones(M,1);     %lower bound vector of x

options=optimset('Display','Iter');

tic
%perform optimization
[x,fval,exitflag] = fmincon(@OptAirfoil,x0,[],[],[],[],lb,ub,[],options);



t=toc;

M_break=M/2;
X_vect = linspace(0,1,99)';      %points for evaluation along x-axis
Aupp_vect=x(1:end);
Alow_vect=x(1:end);
[Xtu,Xtl,C,Thu,Thl,Cm] = D_airfoil2(Aupp_vect,Alow_vect,X_vect);

hold on
plot(Xtu(:,1),Xtu(:,2),'b');    %plot upper surface coords
plot(Xtl(:,1),Xtl(:,2),'b');    %plot lower surface coords
plot(X_vect,C,'ro');                  %plot class function
axis([0,1,-1.5,1.5]);


fid= fopen('withcomb135.dat','r'); % Filename can be changed as required
Coor = fscanf(fid,'%g %g',[2 Inf]) ; 
fclose(fid) ; 

plot(Coor(1,:),Coor(2,:),'rx')
