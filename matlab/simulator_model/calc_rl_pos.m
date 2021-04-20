function [r_leg, r_leg_mid] = calc_rl_pos(i)
    th_rl = pi/180*i;
    Tc0_rl = [cosd(60) 0 -sind(60) -60; -sind(60) 0 -cosd(60) -82;0 1 0 40; 0 0 0 1];
    T01_rl = DHParam(0, -pi/2, 72, th_rl);
    T12_rl = DHParam(122, 0, 0, 0);
    
    
    %joint position
    %baby center
    center = [0,0,0];
    
    %right hip
    r_hip_0 = [0,0,0];
    r_hip_c = Tc0_rl*[r_hip_0,1]';
    r_hip_c = r_hip_c(1:3)';
    %knee in frame 1
    r_knee_pos_1 = [66,0,60];
    %knew transformed to farme 0
    r_knee_pos_c = Tc0_rl*T01_rl*[r_knee_pos_1,1]';
    r_knee_pos_c = r_knee_pos_c(1:3)';
    
    %ankle position in frame 2
    r_ankle_pos_2 = [0,-16,-30];
    %ankle transformed to frame 0
    r_ankle_pos_c  = Tc0_rl*T01_rl*T12_rl*[r_ankle_pos_2,1]';
    r_ankle_pos_c = r_ankle_pos_c(1:3)';
    
    r_leg = [center;r_hip_c;r_knee_pos_c;r_ankle_pos_c];
    
    r_thigh_c = mean([r_hip_c;r_knee_pos_c]);
    r_calf_c = mean([r_knee_pos_c;r_ankle_pos_c]);
    
    r_leg_mid = [center;r_hip_c;r_thigh_c;r_knee_pos_c;r_calf_c;r_ankle_pos_c];

end