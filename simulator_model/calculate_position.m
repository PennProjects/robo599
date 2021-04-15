%%
%for the right hand
l1 = 45; %mm;
l2 = 150;
th_rh = pi/2;

i = 0;
for i = 0:180
    th_rh = pi/180*i;
    Tc0 = [0 0 -1 -75; -1 0 0 75;0 1 0 20; 0 0 0 1];
    T01 = DHParam(0, -pi/2, l1, th_rh);
    T12 = DHParam(l2, 0, 0, 0);
    
    
    %joint position
    %baby center
    center = [0,0,0];
    
    %right shoudler
    r_shoulder_0 = [0,0,0];
    r_shoulder_c = Tc0*[r_shoulder_0,1]';
    r_shoulder_c = r_shoulder_c(1:3)';
    %elbow in frame 1
    r_elbow_pos_1 = [85,0,-18];
    %eblow transformed to farme 0
    r_elbow_pos_c = Tc0*T01*[r_elbow_pos_1,1]';
    r_elbow_pos_c = r_elbow_pos_c(1:3)';
    
    %wrist position in frame 2
    r_wrist_pos_2 = [0,25,45];
    %wrist transformed to frame 0
    r_wrist_pos_c  = Tc0*T01*T12*[wrist_pos_2,1]';
    r_wrist_pos_c = r_wrist_pos_c(1:3)';
    
    r_arm = [center;r_shoulder_c;r_elbow_pos_c;r_wrist_pos_c];
    plot3(r_arm(:,1), r_arm(:,2), r_arm(:,3),'o-','LineWidth', 2,'color','r');
    hold on
    plot3(baby_body(:,1), baby_body(:,2), baby_body(:,3),'o-','LineWidth', 2,'color','black');
    hold off

    
%     view(30,30);
    
    
    xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
    ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
    zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');
    title('Infant simulator forward kinematics')
    
    xlim([-150,150])
    ylim([-200,250])
    zlim([-50,200])
    
    pause(0.001)
    grid on
    drawnow
end

%%
%transformation ffrom r shoulder to baby center
Tc0 = [0 0 -1 -75; -1 0 0 75;0 1 0 20; 0 0 0 1];
r_shoulder_c = Tc0*[r_shoulder_0,1]';
r_shoulder_c = r_shoulder_c(1:3)

%%
baby_body = [-75,75,0;
            75,75,0;
            75,-95,0;
            39,-157,0;
            -39,-157,0;
            -75,-95,0;
            -75,75,0;
            -75,75,60;
            75,75,60;
            75,-95,60;
            39,-157,60;
            -39,-157,60;
            -75,-95,60;
            -75,75,60;
            -75,-95,60;
            -75,-95,0;
            -39,-157,0;
            -39,-157,60;
            39,-157,60;
            39,-157,0;
            75,-95,0;
            75,-95,60;
            75,75,60;
            75,75,0;];
            
            
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