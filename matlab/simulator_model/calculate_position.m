%%
baby_body = [-75,120,0;
            75,120,0;
            75,-50,0;
            39,-112,0;
            -39,-112,0;
            -75,-50,0;
            -75,120,0;
            -75,120,60;
            75,120,60;
            75,-50,60;
            39,-112,60;
            -39,-112,60;
            -75,-50,60;
            -75,120,60;
            -75,-50,60;
            -75,-50,0;
            -39,-112,0;
            -39,-112,60;
            39,-112,60;
            39,-112,0;
            75,-50,0;
            75,-50,60;
            75,120,60;
            75,120,0;];
            
            
plot3(baby_body(:,1), baby_body(:,2), baby_body(:,3),'o-','LineWidth', 2,'color','black');

    
%     view(30,30);


xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');

xlim([-150,150])
ylim([-500,500])
zlim([-50,300])

grid on

%%
%for the right hand
l1 = 45; %mm;
l2 = 150;
th_rh = pi/2;

i = 0;
for i = 0:180
    th_rh = pi/180*i;
    Tc0_rh = [0 0 -1 -75; -1 0 0 90;0 1 0 30; 0 0 0 1];
    T01_rh = DHParam(0, -pi/2, l1, th_rh);
    T12_rh = DHParam(l2, 0, 0, 0);
    
    
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
    plot3(r_arm(:,1), r_arm(:,2), r_arm(:,3),'o-','LineWidth', 2,'color','r');
    hold on
    plot3(baby_body(:,1), baby_body(:,2), baby_body(:,3),'o-','LineWidth', 2,'color','black');
    hold off

    view(0,70);
    
    
    xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
    ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
    zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');
    title('Infant simulator forward kinematics')
    
    xlim([-150,150])
    ylim([-200,250])
    zlim([-50,200])
    
    grid on
    drawnow
end

%%
%right leg 
for i = 0:180
%     i = 0;
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
    
    r_leg = [center;r_hip_c;r_knee_pos_c;r_ankle_pos_c]
    plot3(r_leg(:,1), r_leg(:,2), r_leg(:,3),'o-','LineWidth', 2,'color','b');
    hold on
    plot3(baby_body(:,1), baby_body(:,2), baby_body(:,3),'o-','LineWidth', 2,'color','black');
    hold off

    
    view(30,30);
    
    
    xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
    ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
    zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');
    title('Infant simulator forward kinematics')
    
    xlim([-300,300])
    ylim([-500,250])
    zlim([-100,200])
    
    pause(0.0001)
    grid on
    drawnow
end

%%
%both right hand and right leg
for i = 0:180
    
    %right leg transformations
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
    
    
    %right hand transformations
    th_rh = pi/180*i;
    Tc0_rh = [0 0 -1 -75; -1 0 0 90;0 1 0 30; 0 0 0 1];
    T01_rh = DHParam(0, -pi/2, l1, th_rh);
    T12_rh = DHParam(l2, 0, 0, 0);
    
    
    %joint positions    
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
    
    
    %limb chains
    r_arm = [center;r_shoulder_c;r_elbow_pos_c;r_wrist_pos_c];
    r_leg = [center;r_hip_c;r_knee_pos_c;r_ankle_pos_c];
    
    
    %plotting
    plot3(r_leg(:,1), r_leg(:,2), r_leg(:,3),'o-','LineWidth', 3,'color','b');
    hold on
    plot3(r_arm(:,1), r_arm(:,2), r_arm(:,3),'o-','LineWidth', 3,'color','r');
    plot3(baby_body(:,1), baby_body(:,2), baby_body(:,3),'o-','LineWidth', 2,'color','black');
    text(-10,40,0,'Baby Center')
    text(r_hip_c(1), -200, r_hip_c(3),'Right Hip')
    text(r_shoulder_c(1),200,r_shoulder_c(3),'Right Shoulder')
    hold off

    
    view(0,70);
    
    
    xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
    ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
    zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');
    title('Infant simulator forward kinematics')
    
    xlim([-300,300])
    ylim([-500,250])
    zlim([-100,200])
    
%     pause(0.00001)
    grid on
    drawnow
end

%%
%plotting limbs
for i = 0:5:180
    
    r_hand = calc_rh_pos(i);
    r_leg = calc_rl_pos(180-i);

    l_hand = calc_lh_pos(180-i);
    l_leg = calc_ll_pos(i);
    
%     figure();
    subplot(1,2,1);
    plot3(r_hand(:,1), r_hand(:,2), r_hand(:,3),'o-','LineWidth', 2,'color','r');
    hold on
    plot3(r_leg(:,1), r_leg(:,2), r_leg(:,3),'o-','LineWidth', 2,'color','b');
    plot3(l_hand(:,1), l_hand(:,2), l_hand(:,3),'o-','LineWidth', 2,'color','r');
    plot3(l_leg(:,1), l_leg(:,2), l_leg(:,3),'o-','LineWidth', 2,'color','b');
    plot3(baby_body(:,1), baby_body(:,2), baby_body(:,3),'o-','LineWidth', 2,'color','black');
    
    
    text(-10,40,0,'Baby Center')
    text(r_leg(2,1), -150, r_leg(2,3),'Right Leg')
    text(r_hand(2,1),150,r_hand(3),'Right Hand')
    
    text(l_leg(2,1), -150, l_leg(2,3),'Left Leg')
    text(l_hand(2,1),150, l_hand(2,3),'Left Hand')
    hold off
    
    
    grid on
    view(0,90);
    
    xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
    ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
    zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');
    title('Top view')
    
    xlim([-300,300])
    ylim([-300,250])
    zlim([-30,200])
    
%     drawnow
    
    subplot(1,2,2);
    plot3(r_hand(:,1), r_hand(:,2), r_hand(:,3),'o-','LineWidth', 2,'color','r');
    hold on
    plot3(r_leg(:,1), r_leg(:,2), r_leg(:,3),'o-','LineWidth', 2,'color','b');
    plot3(l_hand(:,1), l_hand(:,2), l_hand(:,3),'o-','LineWidth', 2,'color','r');
    plot3(l_leg(:,1), l_leg(:,2), l_leg(:,3),'o-','LineWidth', 2,'color','b');
    plot3(baby_body(:,1), baby_body(:,2), baby_body(:,3),'o-','LineWidth', 2,'color','black');
    
    
%     text(-10,40,0,'Baby Center')
%     text(r_leg(2,1), -150, r_leg(2,3),'Right Hip')
%     text(r_hand(2,1),150,r_hand(3),'Right Shoulder')
%     
%     text(l_leg(2,1), -150, l_leg(2,3),'Left Hip')
%     text(l_hand(2,1),150, l_hand(2,3),'Left Shoulder')
    hold off

   
    
    grid on
    view(0,35);
    
    xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
    ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
    zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');
    title('Angled View')
    
    xlim([-300,300])
    ylim([-300,250])
    zlim([-30,200])
    
    suptitle('Infant simulator forward kinematics')
    
    drawnow
    
    %     view(-45,60);
%     view(0,90);
%     view(0,60);
    

end

%%
i = 180;

l_leg = calc_ll_pos(i)
plot3(l_leg(:,1), l_leg(:,2), l_leg(:,3),'o-','LineWidth', 2,'color','r');
hold on
plot3(baby_body(:,1), baby_body(:,2), baby_body(:,3),'o-','LineWidth', 2,'color','black');
hold off

view(0,70);


xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');
title('Infant simulator forward kinematics')

xlim([-300,300])
ylim([-300,250])
zlim([-100,200])

grid on
drawnow


%%
%right hand
function [r_arm] = calc_rh_pos(i)
    
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
end

%%
% right leg
function [r_leg] = calc_rl_pos(i)
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

end

%%
%for the left hand
function [l_arm] = calc_lh_pos(i)

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
end

%%
% left leg
function [l_leg] = calc_ll_pos(i)
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

end

    %%
    %function to calculate the tranformation matrix for given set of DH parameters.
    function [hom_trans_matrix] = DHParam(a, alpha, d, theta)
     
%    INPUT:
%       a        - link length, distance along x_i from the intersection of the x_1
%                  and z_i-1 axes to o_i.
%       alpha    - link twist, the angle between z_i-1 and z_i measured about
%                  x_i.
%       d        - link offset, distance along z_i-1 from o_i-1 to the intersection
%                  of the x_i and z_i-1 axes. d_i is variable if joint i is prismatic.
%       theta    - joint angle, the angle between x_i-1 and x_i measured about z_i-1. Theta_i
%                  is variable if joint i is revolute.
%
%   OUTPUT:
%       hom_trans_matrix   - 4 x 4 matrix, a homogenous transformation
%                            matrix that provies the ridig body tranformation, 
%                            including rotation and translation from
%                            frame_i to frame_i-1.


%    Calculates DHParams when given four parameters
        hom_trans_matrix = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta); 
                            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                            0, sin(alpha), cos(alpha), d;
                            0, 0, 0, 1];
    end
    
