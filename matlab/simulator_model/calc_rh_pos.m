function [r_arm, r_arm_mid, J] = calc_rh_pos(i)

    %distance magnitudes
    %top view
    l1 = 45;
    l2 = 135;
    l3 = 15;
    
    %side view
    l4 = 20;
    l5 = 80;
    l6 = 45;
    
    %to baby center (between shoulders)
    l7 = 75;
    l8 = 0;
    l9 = 30;
    
    % to mat center
    l10 = 0;
    l11 = 90;
    l12 = 0;
    
    th_rh = pi/180*i;
    Tmc_rh = [1 0 0 l10;0 1 0 l11;0 0 1 l12];    
    Tc1_rh = [0 0 -1 -l7; -1 0 0 l8;0 1 0 l9; 0 0 0 1];
    T12_rh = DHParam(0, -pi/2, l1, th_rh);
    Tc2_rh = Tc1_rh*T12_rh;
    T23_rh = DHParam(l2, 0, 0, 0);
    Tc3_rh = Tc2_rh*T23_rh;
    
    
    
    %joint position
    %baby center
    center = [0,0,0];
    
    %right shoudler
    r_shoulder_1 = [0,0,0];
    r_shoulder_c = Tc1_rh*[r_shoulder_1,1]';
    r_shoulder_c = r_shoulder_c(1:3)';
    %elbow in frame 2
    r_elbow_pos_2 = [l5,0,-l4];
    %eblow transformed to farme c
    r_elbow_pos_c = Tc2_rh*[r_elbow_pos_2,1]';
    r_elbow_pos_c = r_elbow_pos_c(1:3)';
    
    %wrist position in frame 3
    r_wrist_pos_3 = [0,l3,l6];
    %wrist transformed to frame c
    r_wrist_pos_c  = Tc3_rh*[r_wrist_pos_3,1]';
    r_wrist_pos_c = r_wrist_pos_c(1:3)';
    
    r_arm_c = [center;r_shoulder_c;r_elbow_pos_c;r_wrist_pos_c];
    
    %moving to mat ref
    r_arm_m = (Tmc_rh*[r_arm_c,ones(4,1)]')';
    r_arm_m = r_arm_m(:,1:3);
    
    r_bicep_c = mean([r_shoulder_c; r_elbow_pos_c]);
    r_forearm_c = mean([r_elbow_pos_c;r_wrist_pos_c]);
    
    r_arm_mid_c = [center;r_shoulder_c;r_bicep_c;r_elbow_pos_c;r_forearm_c;r_wrist_pos_c];
    
    %moving to mat ref
    r_arm_mid_m = (Tmc_rh*[r_arm_mid_c,ones(6,1)]')';
    r_arm_mid_m = r_arm_mid_m(:,1:3);
    
    r_arm = r_arm_m;
    r_arm_mid = r_arm_mid_m;
    
    %calculating Jacobian
    J = calculate_Jacobian(Tc1_rh, Tc2_rh, Tc3_rh, r_arm);
   
end

