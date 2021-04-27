function [J] = calculate_Jacobian(Tc1, Tc2, Tc3, joint_pos)


%Calculating the Zaxis of frame i wrt to Frame 0
z_c_i = [[0 0 1];
          Tc1([9 10 11]);
          Tc2([9 10 11]);
          Tc3([9 10 11])];
      

%Calculating the locations of each joint in the Base Frame
jointPositions = joint_pos(1:end,1:3);

joint = 4;
%Calculating manipulator Jacobian upto joint of interest.
for i = 2:joint
    %Using formuala for Revolute joints : Jv = Z_i-1 x (O_n - O_i-1)
    J_v(:,i) = cross(z_c_i(i-1,:),(jointPositions(joint,:)-jointPositions(i-1,:)))';
    
    %Using formuala for Revolute joints : Jw = Z_i-1 x (O_n - O_i-1)
    J_w(:,i) = (z_c_i(i-1,:))';
    
end
%Constructing the manipulator Jacobian J and body velocity xi
J = [J_v; J_w];


end