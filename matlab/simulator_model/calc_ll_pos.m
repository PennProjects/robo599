function [l_leg,l_leg_mid] = calc_ll_pos(i)
    th_ll = pi/180*i;
    Tc0_ll = [-cosd(60) 0 sind(60) 60; -sind(60) 0 -cosd(60) -82;0 -1 0 40; 0 0 0 1];
    T01_ll = DHParam(0, pi/2, 72, -th_ll);
    T12_ll = DHParam(122, 0, 0, 0);
    
    
    %joint position
    %baby center
    center = [0,0,0];
    
    %right hip
    l_hip_0 = [0,0,0];
    l_hip_c = Tc0_ll*[l_hip_0,1]';
    l_hip_c = l_hip_c(1:3)';
    %knee in frame 1
    l_knee_pos_1 = [66,0,60];
    %knew transformed to farme 0
    l_knee_pos_c = Tc0_ll*T01_ll*[l_knee_pos_1,1]';
    l_knee_pos_c = l_knee_pos_c(1:3)';
    
    %ankle position in frame 2
    l_ankle_pos_2 = [0,16,-30];
    %ankle transformed to frame 0
    l_ankle_pos_c  = Tc0_ll*T01_ll*T12_ll*[l_ankle_pos_2,1]';
    l_ankle_pos_c = l_ankle_pos_c(1:3)';
    
    l_leg = [center;l_hip_c;l_knee_pos_c;l_ankle_pos_c];
    
    l_thigh_c = mean([l_hip_c;l_knee_pos_c]);
    l_calf_c = mean([l_knee_pos_c;l_ankle_pos_c]);
    
    l_leg_mid = [center;l_hip_c;l_thigh_c;l_knee_pos_c;l_calf_c;l_ankle_pos_c];

end