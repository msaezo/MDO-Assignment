function [Res] = MDA(sweep,b,lambda_in,lambda_out,root,epsilon_in,epsilon_out,CST,W_wing_c,W_fuel_c,error_n)
%MDA coordinator (convergence loop) for the wing problem
%returns the values for MTOW and W_fuel computed by the disciplinary analyses
%additionally returns the value of the counter and the value of the copy of
%MTOW and W_fuel
%once the convergence is within the specified tolerance (optional)


%for initiation of loop condition:
W_wing = W_wing_c;
W_fuel = W_fuel_c;
counter = 0;
n=1;
while (counter == 0 || (abs(W_fuel-W_fuel_c)/W_fuel>error_n))
    %loop counter
    if (counter > 15)
        W_fuel_c = (W_fuel+W_fuel_c)/2;      
        break
    end
    if (counter > 0)
        W_fuel = W_fuel_c;
    end
    
    Loads(sweep,b,lambda_in,lambda_out,root,epsilon_in,epsilon_out,CST,W_fuel,W_wing);
    write_init(sweep,b,lambda_in,lambda_out,root,epsilon_in,epsilon_out,W_fuel,W_wing);
    
    EMWET Fokker70test;
    fid = fopen('Fokker70test.weight','r');
    numLines = 1;
    your_text = cell(numLines,1);
    for ii = 1:numLines
        your_text(ii) = {fgetl(fid)};
    end
    your_text = char(your_text);
    A = your_text(23:end);
    
    W_wing = str2num(A)*9.81
    fclose(fid);
    
    if isnan(W_wing) == 1
        %say that the wing is very bad so fmincon will look away from this
        %point
        W_wing = 10000*9.81;
        W_fuel_c = 10000*9.81;
        break
    end
    
    [CL_w,CD_w] = Aerodynamics(sweep,b,lambda_in,lambda_out,root,epsilon_in,epsilon_out,CST,W_fuel,W_wing);
    W_fuel_c = Performance(W_fuel,W_wing,CL_w,CD_w);
    n = n + 1;
    
    counter = counter +1;   

end

global couplings;
couplings.W_fuel = W_fuel_c;
couplings.W_wing = W_wing;

% planform(sweep,b,lambda_in,lambda_out, root, 1)
Res = [W_fuel_c,W_wing,counter];

end

