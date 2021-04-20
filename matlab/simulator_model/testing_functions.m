%plotting limbs
for i = 0:5:180
    
    [~,r_hand] = calc_rh_pos(i);
    [~,r_leg] = calc_rl_pos(180-i);

    [~,l_hand] = calc_lh_pos(180-i);
    [~,l_leg] = calc_ll_pos(i);
    baby_body = baby_body_points();
    
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
%calculation of CoM in 3D space

%assigning weight sto each limb section

mass_torso_gm = 1500;
%right side
mass_rh_arm_gm = 70;
mass_rh_forearm_gm = 70;
mass_rl_thigh_gm = 120;
mass_rl_calf_gm = 120;
%left side
mass_lh_arm_gm = 70;
mass_lh_forearm_gm = 70;
mass_ll_thigh_gm = 120;
mass_ll_calf_gm = 120;


%getting joint positions
i = 0;
[~,r_hand] = calc_rh_pos(i);
[~,r_leg] = calc_rl_pos(i);

[~,l_hand] = calc_lh_pos(i);
[~,l_leg] = calc_ll_pos(i);














