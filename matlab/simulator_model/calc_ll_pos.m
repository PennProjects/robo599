function [l_leg,l_leg_mid] = calc_ll_pos(i)

    %distance magnitudes
    %top view
    l1 = 85;
    l2 = 105;
    l3 = 30;
    
    %side view
    l4 = 55;
    l5 = 45;
    l6 = 30;
    
    %to baby center (between shoulders)
    l7 = 60;
    l8 = 172;
    l9 = 40;
    
    % to mat center
    l10 = 0;
    l11 = 90;
    l12 = 0;
    
    th_ll = pi/180*i;
    Tmc_ll = [1 0 0 l10;0 1 0 l11;0 0 1 l12]; 
    Tc1_ll = [-cosd(60) 0 sind(60) l7; -sind(60) 0 -cosd(60) -l8;0 -1 0 l9; 0 0 0 1];
    T12_ll = DHParam(0, pi/2, l1, -th_ll);
    Tc2_ll = Tc1_ll*T12_ll;
    T23_ll = DHParam(l2, 0, 0, 0);
    Tc3_ll = Tc2_ll*T23_ll;
    
    
    %joint position
    %baby center
    center = [0,0,0];
    
    %right hip
    l_hip_1 = [0,0,0];
    l_hip_c = Tc1_ll*[l_hip_1,1]';
    l_hip_c = l_hip_c(1:3)';
    %knee in frame 2
    l_knee_pos_2 = [l5,0,l4];
    %knew transformed to farme c
    l_knee_pos_c = Tc2_ll*[l_knee_pos_2,1]';
    l_knee_pos_c = l_knee_pos_c(1:3)';
    
    %ankle position in frame 3
    l_ankle_pos_3 = [0,-l3,-l6];
    %ankle transformed to frame c
    l_ankle_pos_c  = Tc3_ll*[l_ankle_pos_3,1]';
    l_ankle_pos_c = l_ankle_pos_c(1:3)';
    
    l_leg_c = [center;l_hip_c;l_knee_pos_c;l_ankle_pos_c];
    
    %moving to mat ref
    l_leg_m = (Tmc_ll*[l_leg_c,ones(4,1)]')';
    l_leg_m = l_leg_m(:,1:3);
    
    l_thigh_c = mean([l_hip_c;l_knee_pos_c]);
    l_calf_c = mean([l_knee_pos_c;l_ankle_pos_c]);
    
    l_leg_mid_c = [center;l_hip_c;l_thigh_c;l_knee_pos_c;l_calf_c;l_ankle_pos_c];
    
    %moving to mat ref
    l_leg_mid_m = (Tmc_ll*[l_leg_mid_c,ones(6,1)]')';
    l_leg_mid_m = l_leg_mid_m(:,1:3);
    
    l_leg = l_leg_m;
    l_leg_mid = l_leg_mid_m;
    
    %calculating Jacobian
    J = calculate_Jacobian(Tc1_ll, Tc2_ll, Tc3_ll, l_leg);

end