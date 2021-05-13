%calculation of CoM in 3D space

%assigning weight sto each limb section

mass_torso_gm = 1500;
%right side
mass_rh_arm_gm = 70;
mass_rh_forearm_gm = 70;
mass_rh_gm = mass_rh_arm_gm + mass_rh_forearm_gm;
mass_rl_thigh_gm = 120;
mass_rl_calf_gm = 120;
mass_rl_gm = mass_rl_thigh_gm + mass_rl_calf_gm;

%left side
mass_lh_arm_gm = 70;
mass_lh_forearm_gm = 70;
mass_lh_gm = mass_lh_arm_gm + mass_lh_forearm_gm;
mass_ll_thigh_gm = 120;
mass_ll_calf_gm = 120;
mass_ll_gm = mass_ll_thigh_gm + mass_ll_calf_gm;

total_limb_mass = (mass_rh_gm +mass_lh_gm + mass_rl_gm + mass_ll_gm);

baby_body = baby_body_points(); 

%getting joint positions
for i = 0:5:145
    
    rh = i;
    lh = 0;
    rl = 0;
    ll = 0;
    [com]=calc_com(rh,lh, rl,ll);
    com_x = com(1);
    com_y = com(2);
    com_z = com(3);
    
    %calculating joint postion for given input angle
    [~,r_hand] = calc_rh_pos(rh);
    [~,r_leg] = calc_rl_pos(rl);
    
    
    
    [~,l_hand] = calc_lh_pos(lh);
    [~,l_leg] = calc_ll_pos(ll);
    
        
    %plotting
    %plot 3D simulator model
    subplot(1,2,1);
    plot3(r_hand(:,1), r_hand(:,2), r_hand(:,3),'o-','LineWidth', 2,'color','r');
    hold on
    plot3(r_leg(:,1), r_leg(:,2), r_leg(:,3),'o-','LineWidth', 2,'color','b');
    plot3(l_hand(:,1), l_hand(:,2), l_hand(:,3),'o-','LineWidth', 2,'color','r');
    plot3(l_leg(:,1), l_leg(:,2), l_leg(:,3),'o-','LineWidth', 2,'color','b');
    plot3(baby_body(:,1), baby_body(:,2), baby_body(:,3),'o-','LineWidth', 2,'color','black');
    
    hold off
    grid on
    view(0,70);
    
    xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
    ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
    zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');
    title('Angled View')
    
    xlim([-300,300])
    ylim([-300,250])
    zlim([-30,200])
    
    %plot COM
    subplot(1,2,2);
    plot3(com_x,com_y, com_z, 'o', 'color','r')
    hold on
    xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
    ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
    zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');
    title('COM X-Y plane')
    xlim([-100,100])
    ylim([-200,100])
    view(0,90);

    grid on
    
    drawnow
    
    
end