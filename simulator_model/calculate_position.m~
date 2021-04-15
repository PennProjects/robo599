%%
%for the right hand
l1 = 45; %mm;
l2 = 150;
th_rh = pi/2;

% i = 0;
for i = 0:180
    th_rh = pi/180*i;
    T01 = DHParam(0, -pi/2, l1, th_rh);
    T12 = DHParam(l2, 0, 0, 0);
    
    
    %joint position
    r_shoulder_0 = [0,0,0];
    %elbow in frame 1
    r_elbow_pos_1 = [85,0,-18];
    %eblow transformed to farme 0
    r_elbow_pos_0 = T01*[r_elbow_pos_1,1]';
    r_elbow_pos_0 = r_elbow_pos_0(1:3)';
    
    %wrist position in frame 2
    r_wrist_pos_2 = [0,25,45];
    %wrist transformed to frame 0
    r_wrist_pos_0  = T01*T12*[wrist_pos_2,1]';
    r_wrist_pos_0 = r_wrist_pos_0(1:3)';
    
    r_arm = [r_shoulder_0;r_elbow_pos_0;r_wrist_pos_0];
    plot3(r_arm(:,1), r_arm(:,2), r_arm(:,3),'o-');
    hold on;
    grid on;
    view(3);
    alpha(0.3);
    camup([0 1 0])
    
    xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
    ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
    zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');
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