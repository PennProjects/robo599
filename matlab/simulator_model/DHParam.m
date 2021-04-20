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