function [l_arm, l_arm_mid, J] = calc_lh_pos(i)

    th_lh = pi/180*i;
    Tc1_lh = [0 0 1 75; -1 0 0 90;0 -1 0 30; 0 0 0 1];
    T12_lh = DHParam(0, pi/2, 45, -th_lh);
    Tc2_lh = Tc1_lh*T12_lh;
    T23_lh = DHParam(150, 0, 0, 0);
    Tc3_lh = Tc2_lh*T23_lh;
    
    
    %joint position
    %baby center
    center = [0,0,0];
    
    %left shoudler
    l_shoulder_1 = [0,0,0];
    l_shoulder_c = Tc1_lh*[l_shoulder_1,1]';
    l_shoulder_c = l_shoulder_c(1:3)';
    %elbow in frame 2
    l_elbow_pos_2 = [85,0,-18];
    %eblow transformed to farme c
    l_elbow_pos_c = Tc2_lh*[l_elbow_pos_2,1]';
    l_elbow_pos_c = l_elbow_pos_c(1:3)';
    
    %wrist position in frame 3
    l_wrist_pos_3 = [0,-25,45];
    %wrist transformed to frame c
    l_wrist_pos_c  = Tc3_lh*[l_wrist_pos_3,1]';
    l_wrist_pos_c = l_wrist_pos_c(1:3)';
    
    l_arm = [center;l_shoulder_c;l_elbow_pos_c;l_wrist_pos_c];
    
    l_bicep_c = mean([l_shoulder_c;l_elbow_pos_c]);
    l_forearm_c = mean([l_elbow_pos_c;l_wrist_pos_c]);
    
    l_arm_mid = [center;l_shoulder_c;l_bicep_c;l_elbow_pos_c;l_forearm_c;l_wrist_pos_c];
    
    
    %calculating Jacobian
    J = calculate_Jacobian(Tc1_lh, Tc2_lh, Tc3_lh, l_arm);
end