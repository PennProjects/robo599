function [r_leg, r_leg_mid, J] = calc_rl_pos(i)

    %distance magnitudes
    %top view
    l1 = 75;
    l2 = 125;
    l3 = 15;
    
    %side view
    l4 = 55;
    l5 = 55;
    l6 = 30;
    
    %to center
    l7 = 60;
    l8 = 82;
    l9 = 40;
    
    th_rl = pi/180*i;
    Tc1_rl = [cosd(60) 0 -sind(60) -l7; -sind(60) 0 -cosd(60) -l8;0 1 0 l9; 0 0 0 1];
    T12_rl = DHParam(0, -pi/2, l1, th_rl);
    Tc2_rl = Tc1_rl*T12_rl;
    T23_rl = DHParam(l2, 0, 0, 0);
    Tc3_rl = Tc2_rl*T23_rl;
    
    
    %joint position
    %baby center
    center = [0,0,0];
    
    %right hip
    r_hip_1 = [0,0,0];
    r_hip_c = Tc1_rl*[r_hip_1,1]';
    r_hip_c = r_hip_c(1:3)';
    %knee in frame 2
    r_knee_pos_2 = [l5,0,l4];
    %knew transformed to farme c
    r_knee_pos_c = Tc2_rl*[r_knee_pos_2,1]';
    r_knee_pos_c = r_knee_pos_c(1:3)';
    
    %ankle position in frame 3
    r_ankle_pos_3 = [0,l3,-l6];
    %ankle transformed to frame c
    r_ankle_pos_c  = Tc3_rl*[r_ankle_pos_3,1]';
    r_ankle_pos_c = r_ankle_pos_c(1:3)';
    
    r_leg = [center;r_hip_c;r_knee_pos_c;r_ankle_pos_c];
    
    r_thigh_c = mean([r_hip_c;r_knee_pos_c]);
    r_calf_c = mean([r_knee_pos_c;r_ankle_pos_c]);
    
    r_leg_mid = [center;r_hip_c;r_thigh_c;r_knee_pos_c;r_calf_c;r_ankle_pos_c];
    
    
    %calculating Jacobian
    J = calculate_Jacobian(Tc1_rl, Tc2_rl, Tc3_rl, r_leg);

end