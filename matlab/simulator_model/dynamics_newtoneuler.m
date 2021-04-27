%calculation of Dynamics

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

%%
%testing Newton-euler torque calculations

%get jacobians for input angles
i = 0; %in degrees
[~,~, j_rh] = calc_rh_pos(i);
[~,~, j_rl] = calc_rl_pos(i);

[~,~, j_lh] = calc_lh_pos(0);
[~,~, j_ll] = calc_ll_pos(0);


%%torque = J'*Force

