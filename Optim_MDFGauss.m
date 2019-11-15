function [f,vararg] = Optim_MDFGauss(x)
    global couplings;
    X0 = couplings.X0;
    W_wing_i = couplings.W_wing_c;
    W_fuel_i = couplings.W_fuel_c;
    x = x.*abs(X0);
    sweep = x(1);
    b = x(2);
    lambda_in = x(3);
    lambda_out = x(4);
    root = x(5);
    epsilon_in = x(6);
    epsilon_out = x(7);
    
    CST = [x(8),x(9),x(10),x(11),x(12),x(13),x(14),x(15),...
        x(16),x(17),x(18),x(19),x(20),x(21),x(22),x(23),x(24),x(25),...
        x(26),x(27),x(28),x(29),x(30),x(31)];
    Au_root = [x(8),x(9),x(10),x(11),x(12),x(13)];
    Al_root = [x(14),x(15),x(16),x(17),x(18),x(19)];
    
    Au_tip = [x(20),x(21),x(22),x(23),x(24),x(25)];
    Al_tip = [x(26),x(27),x(28),x(29),x(30),x(31)];
    
    [Xtu_root,Xtl_root] = D_airfoil2(Au_root,Al_root);
    [Xtu_tip,Xtl_tip] = D_airfoil2(Au_tip,Al_tip);
    

        %Write on airfoil
    
    fid = fopen('Airfoil_root.dat','wt');
    for ii = 1:size(Xtu_root,1)
        fprintf(fid, '%g\t',Xtu_root(ii,:));
        fprintf(fid,'\n');
    end
    for ii = 1:size(Xtl_root,1)
        fprintf(fid, '%g\t',Xtl_root(ii,:));
        fprintf(fid,'\n');
    end
    fclose(fid);
    
        fid = fopen('Airfoil_tip.dat','wt');
    for ii = 1:size(Xtu_tip,1)
        fprintf(fid, '%g\t',Xtu_tip(ii,:));
        fprintf(fid,'\n');
    end
    for ii = 1:size(Xtl_tip,1)
        fprintf(fid, '%g\t',Xtl_tip(ii,:));
        fprintf(fid,'\n');
    end
    fclose(fid);
    
%     global airfoil;
%     
%     subplot(3,2,1);
%     xlim([0 1])
%     ylim([-0.5 0.5])
%     plot(Xtu_tip(:,1),Xtu_tip(:,2))
%     hold on    
%     xlim([0 1])
%     ylim([-0.5 0.5])
%     plot(Xtl_tip(:,1),Xtl_tip(:,2))
%     title(' NEW Tip')
%     hold off
%     
%     subplot(3,2,2);
%     xlim([0 1])
%     ylim([-0.5 0.5])
%     plot(airfoil.X0tu_tip(:,1),airfoil.X0tu_tip(:,2))
%     hold on     
%     xlim([0 1])
%     ylim([-0.5 0.5])
%     plot(airfoil.X0tl_tip(:,1),airfoil.X0tl_tip(:,2))
%     title('INITIAL Tip')
%     hold off
%     subplot(3,2,3);
%     xlim([0 1])
%     ylim([-0.5 0.5])
%     plot(Xtu_root(:,1),Xtu_root(:,2))
%     hold on
%     xlim([0 1])
%     ylim([-0.5 0.5])
%     plot(Xtl_root(:,1),Xtl_root(:,2))
%     title('NEW root')
%     hold off
%     
%     subplot(3,2,4);
%     xlim([0 1])
%     ylim([-0.5 0.5])
%     plot(airfoil.X0tu_root(:,1),airfoil.X0tu_root(:,2))
%     hold on    
%     xlim([0 1])
%     ylim([-0.5 0.5])
%     plot(airfoil.X0tl_root(:,1),airfoil.X0tl_root(:,2))
%     title('INITIAL root')
%     hold off
%    
%     subplot(3,2,5);
%     planform(sweep,b,lambda_in,lambda_out, root, 1)
%     title('NEW planform')
%     
%     subplot(3,2,6);
%     planform(24,28.08,0.6,0.4,5.8, 1)
%     title('INITIAL planform')
 
    %Initial guess for output of discipline 2
    error_n = 10^(-2);
    %call MDA coordinator
    Res = MDA(sweep,b,lambda_in,lambda_out,root,epsilon_in,epsilon_out,CST,W_wing_i,W_fuel_i,error_n);
    
    W_fuel = Res(1);
    W_wing= Res(2);
    counter= Res(3);
    
    f = Objective(W_fuel,W_wing);
    vararg = {W_fuel,W_wing,counter};
end