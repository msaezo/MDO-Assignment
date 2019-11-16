function[V_tank_tot] = VolumeTank(x)
%This function calculates the volume available for the fuel in the wing. It
%bases the calculation on volume integrals over the surface area of the
%airfoil available for the tank.
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
    
    [Xtu_root_ini,Xtl_root_ini] = D_airfoil2(Au_root,Al_root);
    %[Xtu_tip,Xtl_tip] = D_airfoil2(Au_tip,Al_tip);
    
    Xtu_root = Xtu_root_ini.*root;
    Xtl_root = Xtl_root_ini.*root;
    Xtl_root = flip(Xtl_root);
    Xtl_root = [Xtl_root(:,1),Xtl_root(:,2)*(-1)];
    
    %Set parameters of tank position (chordwise)
    tank_chord_front = 0.15;
    tank_chord_back = 0.60;
    tank_length_ratio = tank_chord_back - tank_chord_front ;
    step_size = 0.01;
    
    %Set parameters of tank position (spanwise)
    tank_loc_out = 0.85;
    tank_loc_in = 5.616/(b/2);
    
    
    %Set parameters of t/c ratio of airfoil root vs tip
    ratio_tc_root_tip = 0.6;
    slope_tc = ratio_tc_root_tip - 1;
    
    %Calculate the height of the root airfoil at the span locations
    spar_front_upp_height = interp1(Xtu_root(:,1),Xtu_root(:,2),tank_chord_front*root);
    spar_back_upp_height = interp1(Xtu_root(:,1),Xtu_root(:,2),tank_chord_back*root);
    spar_front_low_height = interp1(Xtl_root(:,1),Xtl_root(:,2),tank_chord_front*root);
    spar_back_low_height = interp1(Xtl_root(:,1),Xtl_root(:,2),tank_chord_back*root);
    spar_front_height = (spar_front_upp_height - spar_front_low_height);
    spar_back_height = (spar_back_upp_height - spar_back_low_height);
    
    %integration over wing of inner tank
    n = 1; %For each span position
    b = b/2; %Make the span half, later multiplied by 2
    
    
    for b_sec = 0:step_size:b*tank_loc_out
        
        if b_sec <= 5.37
            %Per airfoil section calculate the chord
            delta_chord = ((root*lambda_in)-(root))/(b*tank_loc_in);
            chord_sec(n) = root + delta_chord*b_sec;
            %Per airfoil section calculate the length of the chord for the tank
            l_tank = chord_sec(n)*tank_length_ratio;
            %Per airfoil section calculate the height of the tank
            delta_height = (1-slope_tc*(b_sec/b));
            height_front_sec(n) = spar_front_height*delta_height;
            height_back_sec(n) = spar_back_height*delta_height;
            %Calculate the area of the airfoil
            area_tank_sec(n) = l_tank*((height_front_sec(n) + height_back_sec(n))/2);
        else
            %Per airfoil section calculate the chord
            delta_chord = ((root*lambda_in*lambda_out)-(root*lambda_in))/(b*(tank_loc_out-tank_loc_in));
            chord_sec(n) = root*lambda_in + delta_chord*b_sec;
            %Per airfoil section calculate the length of the chord for the tank
            l_tank = chord_sec(n)*tank_length_ratio;
            %Per airfoil section calculate the height of the tank
            delta_height = (1-slope_tc*(b_sec/b));
            height_front_sec(n) = spar_front_height*delta_height;
            height_back_sec(n) = spar_back_height*delta_height;
            %Calculate the area of the airfoil
            area_tank_sec(n) = l_tank*((height_front_sec(n) + height_back_sec(n))/2);
           
        end
        n = n + 1;
    end
    
    %Calculate the Volume of the tank
    V_tank = 0;
    n = 1;
    %integration over wing of outer tank
    for b_sec = 0:step_size:b*tank_loc_out
        V_tank = V_tank + step_size*area_tank_sec(n);
        n = n + 1;
    end
    %Multiply tank volume by two, two half wings
    V_tank_tot = V_tank*2;
end