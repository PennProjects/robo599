function [l_arm, l_arm_mid, J] = calc_lh_pos(i)
    
    %distance magnitudes
    %top view
    l1 = 45;
    l2 = 115;
    l3 = 25;
    
    %side view
    l4 = 8;
    l5 = 70;
    l6 = 55;
    
    %to baby center (between shoulders)
    l7 = 75;
    l8 = 0;
    l9 = 30;

    % to mat center
    l10 = 0;
    l11 = 90;
    l12 = 0;
    
    th_lh = pi/180*i;
    Tmc_lh = [1 0 0 l10;0 1 0 l11;0 0 1 l12]; 
    Tc1_lh = [0 0 1 l7; -1 0 0 l8;0 -1 0 l9; 0 0 0 1];
    T12_lh = DHParam(0, pi/2, l1, -th_lh);
    Tc2_lh = Tc1_lh*T12_lh;
    T23_lh = DHParam(l2, 0, 0, 0);
    Tc3_lh = Tc2_lh*T23_lh;
    
    
    %joint position
    %baby center
    center = [0,0,0];
    
    %left shoudler
    l_shoulder_1 = [0,0,0];
    l_shoulder_c = Tc1_lh*[l_shoulder_1,1]';
    l_shoulder_c = l_shoulder_c(1:3)';
    %elbow in frame 2
    l_elbow_pos_2 = [l5,0,-l4];
    %eblow transformed to farme c
    l_elbow_pos_c = Tc2_lh*[l_elbow_pos_2,1]';
    l_elbow_pos_c = l_elbow_pos_c(1:3)';
    
    %wrist position in frame 3
    l_wrist_pos_3 = [0,-l3,l6];
    %wrist transformed to frame c
    l_wrist_pos_c  = Tc3_lh*[l_wrist_pos_3,1]';
    l_wrist_pos_c = l_wrist_pos_c(1:3)';
    
    l_arm_c = [center;l_shoulder_c;l_elbow_pos_c;l_wrist_pos_c];
    
    %moving to mat ref
    l_arm_m = (Tmc_lh*[l_arm_c,ones(4,1)]')';
    l_arm_m = l_arm_m(:,1:3);
    
    l_bicep_c = mean([l_shoulder_c;l_elbow_pos_c]);
    l_forearm_c = mean([l_elbow_pos_c;l_wrist_pos_c]);
    
    l_arm_mid_c = [center;l_shoulder_c;l_bicep_c;l_elbow_pos_c;l_forearm_c;l_wrist_pos_c];
    
    %moving to mat ref
    l_arm_mid_m = (Tmc_lh*[l_arm_mid_c,ones(6,1)]')';
    l_arm_mid_m = l_arm_mid_m(:,1:3);
    
    l_arm = l_arm_m;
    l_arm_mid = l_arm_mid_m;
    
    
    %calculating Jacobian
    J = calculate_Jacobian(Tc1_lh, Tc2_lh, Tc3_lh, l_arm);
end