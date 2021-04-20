function [l_arm, l_arm_mid] = calc_lh_pos(i)

    th_lh = pi/180*i;
    Tc0_lh = [0 0 1 75; -1 0 0 90;0 -1 0 30; 0 0 0 1];
    T01_lh = DHParam(0, pi/2, 45, -th_lh);
    T12_lh = DHParam(150, 0, 0, 0);
    
    
    %joint position
    %baby center
    center = [0,0,0];
    
    %left shoudler
    l_shoulder_0 = [0,0,0];
    l_shoulder_c = Tc0_lh*[l_shoulder_0,1]';
    l_shoulder_c = l_shoulder_c(1:3)';
    %elbow in frame 1
    l_elbow_pos_1 = [85,0,-18];
    %eblow transformed to farme 0
    l_elbow_pos_c = Tc0_lh*T01_lh*[l_elbow_pos_1,1]';
    l_elbow_pos_c = l_elbow_pos_c(1:3)';
    
    %wrist position in frame 2
    l_wrist_pos_2 = [0,-25,45];
    %wrist transformed to frame 0
    l_wrist_pos_c  = Tc0_lh*T01_lh*T12_lh*[l_wrist_pos_2,1]';
    l_wrist_pos_c = l_wrist_pos_c(1:3)';
    
    l_arm = [center;l_shoulder_c;l_elbow_pos_c;l_wrist_pos_c];
    
    l_bicep_c = mean([l_shoulder_c;l_elbow_pos_c]);
    l_forearm_c = mean([l_elbow_pos_c;l_wrist_pos_c]);
    
    l_arm_mid = [center;l_shoulder_c;l_bicep_c;l_elbow_pos_c;l_forearm_c;l_wrist_pos_c];
end