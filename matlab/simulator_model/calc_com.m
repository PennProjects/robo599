function [com] = calc_com(rh,lh, rl, ll)
%Return com for given angle

mass_torso_gm = 1500;
%right side
mass_rh_arm_gm = 70;
mass_rh_forearm_gm = 70;
mass_rh_gm = mass_rh_arm_gm + mass_rh_forearm_gm;
mass_rl_thigh_gm = 120;
mass_rl_calf_gm = 120;
mass_rl_gm = mass_rl_thigh_gm + mass_rl_calf_gm;

%left side
mass_lh_arm_gm = 70;
mass_lh_forearm_gm = 70;
mass_lh_gm = mass_lh_arm_gm + mass_lh_forearm_gm;
mass_ll_thigh_gm = 120;
mass_ll_calf_gm = 120;
mass_ll_gm = mass_ll_thigh_gm + mass_ll_calf_gm;

total_limb_mass = (mass_rh_gm +mass_lh_gm + mass_rl_gm + mass_ll_gm);

%calculating joint postion for given input angle
    [~,r_hand] = calc_rh_pos(rh);
    [~,r_leg] = calc_rl_pos(rl);
    
    
    
    [~,l_hand] = calc_lh_pos(lh);
    [~,l_leg] = calc_ll_pos(ll);

    
    
    
    %calculating com_x = sum(x_i*mass_i)/sum(mass_i)
    com_x = (r_hand(3,1)*mass_rh_arm_gm + r_hand(5,1)*mass_rh_forearm_gm +...
            l_hand(3,1)*mass_lh_arm_gm + l_hand(5,1)*mass_lh_forearm_gm +...
            r_leg(3,1)*mass_rl_thigh_gm + r_leg(5,1)*mass_rl_calf_gm +...
            l_leg(3,1)*mass_ll_thigh_gm + l_leg(5,1)*mass_ll_calf_gm)/...
            (total_limb_mass);
            
    %calculating com_y = sum(y_i*mass_i)/sum(mass_i)
    com_y = (r_hand(3,2)*mass_rh_arm_gm + r_hand(5,2)*mass_rh_forearm_gm +...
            l_hand(3,2)*mass_lh_arm_gm + l_hand(5,2)*mass_lh_forearm_gm +...
            r_leg(3,2)*mass_rl_thigh_gm + r_leg(5,2)*mass_rl_calf_gm +...
            l_leg(3,2)*mass_ll_thigh_gm + l_leg(5,2)*mass_ll_calf_gm)/...
            (total_limb_mass);
        
    %calculating com_z = sum(z_i*mass_i)/sum(mass_i)    
    com_z = (r_hand(3,3)*mass_rh_arm_gm + r_hand(5,3)*mass_rh_forearm_gm +...
            l_hand(3,3)*mass_lh_arm_gm + l_hand(5,3)*mass_lh_forearm_gm +...
            r_leg(3,3)*mass_rl_thigh_gm + r_leg(5,3)*mass_rl_calf_gm +...
            l_leg(3,3)*mass_ll_thigh_gm + l_leg(5,3)*mass_ll_calf_gm)/...
            (total_limb_mass);
    com = [com_x, com_y,com_z];
end

