function [r_arm, r_arm_mid] = calc_rh_pos(i)

   
    
    th_rh = pi/180*i;
    Tc0_rh = [0 0 -1 -75; -1 0 0 90;0 1 0 30; 0 0 0 1];
    T01_rh = DHParam(0, -pi/2, 45, th_rh);
    T12_rh = DHParam(150, 0, 0, 0);
    
    
    %joint position
    %baby center
    center = [0,0,0];
    
    %right shoudler
    r_shoulder_0 = [0,0,0];
    r_shoulder_c = Tc0_rh*[r_shoulder_0,1]';
    r_shoulder_c = r_shoulder_c(1:3)';
    %elbow in frame 1
    r_elbow_pos_1 = [85,0,-18];
    %eblow transformed to farme 0
    r_elbow_pos_c = Tc0_rh*T01_rh*[r_elbow_pos_1,1]';
    r_elbow_pos_c = r_elbow_pos_c(1:3)';
    
    %wrist position in frame 2
    r_wrist_pos_2 = [0,25,45];
    %wrist transformed to frame 0
    r_wrist_pos_c  = Tc0_rh*T01_rh*T12_rh*[r_wrist_pos_2,1]';
    r_wrist_pos_c = r_wrist_pos_c(1:3)';
    
    r_arm = [center;r_shoulder_c;r_elbow_pos_c;r_wrist_pos_c];
    
    r_bicep_c = mean([r_shoulder_c; r_elbow_pos_c]);
    r_forearm_c = mean([r_elbow_pos_c;r_wrist_pos_c]);
    
    r_arm_mid = [center;r_shoulder_c;r_bicep_c;r_elbow_pos_c;r_forearm_c;r_wrist_pos_c];
end

