%% load csv
raw_mat_nm_1 = readtable('/Users/jalpanchal/drive/penn/robo599/simulator_media/0429/nomov/mat/2021-4-29_16_51_nomov_1.csv');
raw_mat_rh_1 = readtable('/Users/jalpanchal/drive/penn/robo599/simulator_media/0429/right_hand/mat/2021-4-29_16_52_Rgthnd_1.csv');
raw_mat_rl_1 = readtable('/Users/jalpanchal/drive/penn/robo599/simulator_media/0429/right_leg/mat/2021-4-29_16_55_rgtleg_1.csv');
raw_mat_ll_3 = readtable('/Users/jalpanchal/drive/penn/robo599/simulator_media/0429/left_leg/mat/2021-4-29_17_3_lftleg_3.csv');


%%
%plot raw x vx y
% plot(datset.Var5(3:10,:), datset.Var6(3:10,:), 'o-');
figure();
datset  = raw_mat_nm_1;
for i = 1:size(datset,1)
    plot(datset.Var5(i), datset.Var6(i), 'o-')
    xlim([-20,5])
    ylim([-10,30])
    hold on
    drawnow
end

%%
figure();
datset  = raw_mat_rl_1;
for i = 1:size(datset,1)
    plot(datset.Var5(i), datset.Var6(i), 'o-')
    xlim([-20,5])
    ylim([-10,30])
    hold on
    drawnow
end

%%
subplot(1,2,1)
datset  = raw_mat_rl_1;
plot(datset.Var5, datset.Var6, 'o-')
xlim([-20,5])
ylim([-10,30])

subplot(1,2,2)
datset  = raw_mat_ll_3
plot(datset.Var5, datset.Var6, 'o-')
xlim([-20,5])
ylim([-10,30])
%% Plotting mat data
subplot(1,2,1)
datset  = raw_mat_rl_1;
plot(datset.Var5, datset.Var6, 'o-')
xlim([-20,5])
ylim([-10,30])
title('Right Leg')

subplot(1,2,2)
datset  = raw_mat_ll_3;
plot(datset.Var5, datset.Var6, 'o-')
xlim([-20,5])
ylim([-10,30])
title('Left Leg')


%% Animation
figure();
dataset  = raw_mat_rl_1;
for i = 1:size(dataset,1)
    plot(dataset.Var5(i), dataset.Var6(i), 'o-')
    xlim([-20,5])
    ylim([-10,30])
    hold on
    drawnow
end

%% Calculating COP magnitude magnitude
%Cutting at 135 : 813 to start and stop during movement
limb_data_raw  = raw_mat_rl_1(130:813,:);
mat_x_raw= limb_data_raw.Var5;
mat_y_raw= limb_data_raw.Var6;
cop_mag_mat = vecnorm([mat_x_raw, mat_y_raw]')';

%plotting cop magnitude
plot(cop_mag_mat);

%% Down sampling mat data to match sim data
limb_data_downsamp  = limb_data_raw;
sim_data = raw_sim_rl_1;
limb_data_size = size(limb_data_downsamp,1);
sim_data_size = size(sim_data,1);
mat_x_downsamp = resample(limb_data_downsamp.Var5,sim_data_size,limb_data_size);
mat_y_dowmsamp = resample(limb_data_downsamp.Var6, sim_data_size, limb_data_size);

cop_mag_mat_downsamp = vecnorm([mat_x_downsamp, mat_y_dowmsamp]')';
plot(cop_mag_mat_downsamp)


%% Impact of down sampling
subplot(2,2,1)
dataset  = limb_data_raw;
plot(dataset.Var5, dataset.Var6, 'o-')
xlim([-20,5])
ylim([-10,30])

subplot(2,2,2)
plot(cop_mag_mat)

subplot(2,2,3)
plot(downsam_mat_x, downsam_mat_y, 'o-')
xlim([-20,5])
ylim([-10,30])

subplot(2,2,4)
plot(cop_mag_mat_downsamp)

%% plotting cop vs sim angle
sim_angle_raw  = raw_sim_rl_1.rgtleg;
plot(sim_angle_raw,cop_mag_mat_downsamp)
xlabel('Joing angle(deg)')
ylabel('CoP magnitude')
