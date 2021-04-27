function [l_leg,l_leg_mid] = calc_ll_pos(i)

    th_ll = pi/180*i;
    Tc1_ll = [-cosd(60) 0 sind(60) 60; -sind(60) 0 -cosd(60) -82;0 -1 0 40; 0 0 0 1];
    T12_ll = DHParam(0, pi/2, 72, -th_ll);
    Tc2_ll = Tc1_ll*T12_ll;
    T23_ll = DHParam(122, 0, 0, 0);
    Tc3_ll = Tc2_ll*T23_ll;
    
    
    %joint position
    %baby center
    center = [0,0,0];
    
    %right hip
    l_hip_1 = [0,0,0];
    l_hip_c = Tc1_ll*[l_hip_1,1]';
    l_hip_c = l_hip_c(1:3)';
    %knee in frame 2
    l_knee_pos_2 = [66,0,60];
    %knew transformed to farme c
    l_knee_pos_c = Tc2_ll*[l_knee_pos_2,1]';
    l_knee_pos_c = l_knee_pos_c(1:3)';
    
    %ankle position in frame 3
    l_ankle_pos_3 = [0,16,-30];
    %ankle transformed to frame c
    l_ankle_pos_c  = Tc3_ll*[l_ankle_pos_3,1]';
    l_ankle_pos_c = l_ankle_pos_c(1:3)';
    
    l_leg = [center;l_hip_c;l_knee_pos_c;l_ankle_pos_c];
    
    l_thigh_c = mean([l_hip_c;l_knee_pos_c]);
    l_calf_c = mean([l_knee_pos_c;l_ankle_pos_c]);
    
    l_leg_mid = [center;l_hip_c;l_thigh_c;l_knee_pos_c;l_calf_c;l_ankle_pos_c];
    
    %calculating Jacobian
    J = calculate_Jacobian(Tc1_ll, Tc2_ll, Tc3_ll, l_leg);

end