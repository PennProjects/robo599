%% load csv
main_folder = '/Users/jalpanchal/drive/penn/robo599/simulator_media/';
date = {'0429/'};
limb = {'right_hand/', 'left_hand/','right_leg/', 'left_leg/'};
limb_name = {'Right hand', 'Left Hand', 'Right Leg', 'Left Leg'};
device = {'mat/','simulator/'};
trials = 3;
trial_numbers = [1,2;1,2;1,2;1,3];
%read mat data
for j = 1:4
    path = strcat(main_folder,date{1},limb{j},device{1});
    files = dir(strcat(path,'*.csv'));
    outs = cell(numel(files),1);
    for i = 1:numel(files)
        file_name = strcat(path,files(i).name);
        raw_mat{j,i} = readtable(file_name);
    end
end

%read sim data
for j = 1:4
    path = strcat(main_folder,date{1},limb{j},device{2});
    files = dir(strcat(path,'*.csv'));
    outs = cell(numel(files),1);
    for i = 1:numel(files)  
        file_name = strcat(path,files(i).name);
        raw_sim{j,i} = readtable(file_name);
    end
end

%%
%1-RH, 2-LH, 3-RL, 4-LL
limb_select = 3;

mat_data_alltrials = array2table(zeros(1,4));
sim_data_alltrials = array2table(zeros(1,6));
sim_data_alltrials.Properties.VariableNames = {'time_ms','rgthnd','lfthnd', 'rgtleg', 'lftleg','trial_num'};
load('0429_mat_trunkidx.mat')
trial_time = [];
for t = 1:2
    t_num = trial_numbers(limb_select, t);
    mat_data_raw = raw_mat{limb_select, t_num};
    sim_data_raw = raw_sim{limb_select, t_num};

    %truncating data to experiment
    start_idx = start_idx_mat(limb_select,t_num) ;
    end_idx = end_idx_mat(limb_select,t_num);
    mat_data_trunk  = mat_data_raw(start_idx:end_idx,5:7);

    
    %resampling to 30 Hz
    %total samples
    sim_data_raw.time_ms = sim_data_raw.time_ms - sim_data_raw.time_ms(1);
    trial_time_s = sim_data_raw.time_ms(end)/1e3
    target_samples = ceil(trial_time_s*30);
    
    %resampling mat data    
    mat_data_resamp = mat_data_trunk;
    mat_samples = size(mat_data_trunk,1);
    mat_data_resamp = array2table(resample(table2array(mat_data_trunk),target_samples,mat_samples));
    
    %resample sim data
    sim_data_resamp = sim_data_raw;
    sim_samples = size(sim_data_raw,1);
    sim_data_resamp = array2table(resample(table2array(sim_data_raw),target_samples,sim_samples));

    %Concatinating data from all trials
    trial_number = t_num*ones(size(mat_data_resamp,1),1);
    mat_data_resamp.trial_num = trial_number;
    %renaming variables to facilitate concat
    mat_data_resamp.Properties.VariableNames = {'Var1','Var2','Var3','Var4'};
    mat_data_alltrials = [mat_data_alltrials;mat_data_resamp];
    
    %sim data
    trial_number_ = t_num*ones(size(sim_data_resamp,1),1);
    sim_data_resamp.trial_num = trial_number_;
    sim_data_resamp.Properties.VariableNames = {'time_ms','rgthnd','lfthnd', 'rgtleg', 'lftleg','trial_num'};
    sim_data_alltrials = [sim_data_alltrials;sim_data_resamp];
end
%removing zero entry
mat_data_alltrials(1,:) = [];
sim_data_alltrials(1,:) = [];
%remaming columns
mat_data_alltrials.Properties.VariableNames = {'cop_x','cop_y','r', 'trial_num'};


%%
% Smoothen and filter mat data

%truncated raw data
mat_x_allt= mat_data_alltrials.cop_x;
mat_y_allt= mat_data_alltrials.cop_y;
copmag_mat_allt = vecnorm([mat_x_allt, mat_y_allt]')';
% subplot(3,1,1)
% plot(copmag_mat_allt, 'Linewidth', 2, 'color', [0.6350 0.0780 0.1840])
% ylabel('CoP Magnitude (mm)')
% title('CoP Magnitude from Force Mat')
% 
% subplot(3,1,2)
% plot(mat_x_allt, 'Linewidth', 2, 'color', [0.6350 0.0780 0.1840])
% ylabel('CoP Magnitude (mm)')
% title('CoP X from Force Mat')
% 
% subplot(3,1,3)
% plot(mat_y_allt, 'Linewidth', 2, 'color', [0.6350 0.0780 0.1840])
% ylabel('CoP Magnitude (mm)')
% title('CoP Y from Force Mat')

%testing smoothening functions
%%matlab smoothen function
data_raw = mat_data_alltrials.cop_y;
data_smoothdat_10= smoothdata(data_raw, 'movmean', 10)
data_smoothdat_20= smoothdata(data_raw, 'movmean', 20);
data_smoothdat_30= smoothdata(data_raw, 'movmean', 30);
data_smoothdat_40= smoothdata(data_raw, 'movmean', 40);
data_smoothdat_100= smoothdata(data_raw, 'movmean', 100);


%%sgolay filter
data_sgol = sgolayfilt(data_raw, 4, 89);

cmap = lines(7);
plot(data_raw, 'color', cmap(1,:))
hold on
plot(data_sgol, 'Linewidth', 3, 'color', cmap(2,:))
plot(data_smoothdat_10,'-.','Linewidth', 3,'color', cmap(3,:))
plot(data_smoothdat_20,'-.','Linewidth', 3,'color', cmap(4,:))
plot(data_smoothdat_30,'-.','Linewidth', 3,'color', cmap(5,:))
plot(data_smoothdat_40,'-.','Linewidth', 3,'color', cmap(6,:))
plot(data_smoothdat_100,'-.','Linewidth', 3,'color', cmap(7,:))
% ylim([0,20])
xlim([0,300])

hold off
grid on
legend('raw','sgolay : 4,89', 'smoothdata :10', 'smoothdata :20', 'smoothdata :30', 'smoothdata :40', 'smoothdata :100')
xlabel("Frame Number")
ylabel("COP Y (mm)")
title("Comparing smoothening functions for Mat data. Right Leg,  COP Y data")

corr_raw_smoothdata = corrcoef(data_raw, data_smoothdat_20)
corr_raw_sgolay = corrcoef(data_raw, data_sgol)

%% Smoothen and filter data
mat_data_smoothen = mat_data_alltrials;
%sgolay
sg_order = 4;
sg_framelen = 89;
mat_data_smoothen.cop_x = sgolayfilt(mat_data_alltrials.cop_x,sg_order,sg_framelen);
mat_data_smoothen.cop_y = sgolayfilt(mat_data_alltrials.cop_y,sg_order,sg_framelen);

%smmothdata
% sm_windowlen = 45;
% mat_data_smoothen.cop_x = smoothdata(mat_data_alltrials.cop_x,'movmean',sm_windowlen);
% mat_data_smoothen.cop_y = smoothdata(mat_data_alltrials.cop_y,'movmean',sm_windowlen);


%% Calculating CoP magnitude
mat_data_smoothen.cop_mag = vecnorm([mat_data_smoothen.cop_x, mat_data_smoothen.cop_y]')';
%% fetching Joint positions
sim_angle_raw  = sim_data_alltrials(:,["rgthnd","lfthnd","rgtleg","lftleg"]).Variables;
% time_stamp_s = sim_data_alltrials(:,1).Variables/1e3;
time_stamp_s = 1:size(sim_angle_raw,1);
jointpos_x = [];
jointpos_y = [];

for i = 1:size(sim_angle_raw,1)
    rh = calc_rh_pos(sim_angle_raw(i,1));
    lh = calc_lh_pos(sim_angle_raw(i,2));
    rl = calc_rl_pos(sim_angle_raw(i,3));
    ll = calc_ll_pos(sim_angle_raw(i,4));
    
    jointpos_curr_x_ = [rh(end,1),lh(end,1),rl(end,1),ll(end,1)];
    jointpos_x = [jointpos_x; jointpos_curr_x_];
    
    jointpos_curr_y_ = [rh(end,2),lh(end,2),rl(end,2),ll(end,2)];
    jointpos_y = [jointpos_y; jointpos_curr_y_];
end

%% plotting time series and Mat vs Simulator
figure();

subplot(3,3,1)
plot(time_stamp_s,sim_angle_raw(:,limb_select), 'Linewidth', 2, 'color', [0 0.4470 0.7410])
xlabel('Simulation Time(s)')
ylabel('Joint Angle (deg)')
title('Simulator Joint Angle')

subplot(3,3,2)
plot(time_stamp_s,jointpos_x(:,limb_select), 'Linewidth', 2, 'color', [0 0.4470 0.7410])
xlabel('Simulation Time(s)')
ylabel('End-effector Postion (mm)')
title('End-Effector X Position from Baby Center')

subplot(3,3,3)
plot(time_stamp_s,jointpos_y(:,limb_select), 'Linewidth', 2, 'color', [0 0.4470 0.7410])
xlabel('Simulation Time(s)')
ylabel('End-effector Postion (mm)')
title('End-Effector Y Position from Baby Center')

subplot(3,3,4)
plot(time_stamp_s,mat_data_smoothen.cop_mag, 'Linewidth', 2, 'color', [0.6350 0.0780 0.1840])
xlabel('Simulation Time(s)')
ylabel('CoP Magnitude (mm)')
title('CoP Magnitude from Force Mat')

subplot(3,3,5)
plot(time_stamp_s,mat_data_smoothen.cop_x, 'Linewidth', 2, 'color', [0.6350 0.0780 0.1840])
xlabel('Simulation Time(s)')
ylabel('CoP Magnitude (mm)')
title('CoP X from Force Mat')

subplot(3,3,6)
plot(time_stamp_s,mat_data_smoothen.cop_y, 'Linewidth', 2, 'color', [0.6350 0.0780 0.1840])
xlabel('Simulation Time(s)')
ylabel('CoP Magnitude (mm)')
title('CoP Y from Force Mat')

subplot(3,3,7)
plot(sim_angle_raw(:,limb_select),mat_data_smoothen.cop_mag, 'o', 'Linewidth', 1.5, 'color', [0.9290 0.6940 0.1250])
xlabel('Joint Angle (deg)')
ylabel('CoP Magnitude (mm)')
title('CoP Magnitude vs Joint Angle')

subplot(3,3,8)
plot(jointpos_x(:,limb_select),mat_data_smoothen.cop_x, 'o', 'Linewidth', 1.5, 'color', [0.8500 0.3250 0.0980])
xlabel('End-effector Position (mm)')
ylabel('CoP Magnitude (mm)')
title('CoP Magnitude X vs Joint Position X')

subplot(3,3,9)
plot(jointpos_y(:,limb_select),mat_data_smoothen.cop_y, 'o', 'Linewidth', 1.5, 'color', [0.4660 0.6740 0.1880])
xlabel('End-effector Position (mm)')
ylabel('CoP Magnitude (mm)')
title('CoP Magnitude Y vs Joint Position Y')

suptitle("Comparison of Mat and Simultor data  "+"Limb: "+limb_name{limb_select})


%% plotting time series
% figure();
% 
% subplot(2,3,1)
% plot(time_stamp_s,sim_angle_raw(:,limb_select), 'Linewidth', 2, 'color', [0 0.4470 0.7410])
% xlabel('Simulation Time(s)')
% ylabel('Joint Angle (deg)')
% title('Simulator Joint Angle')
% 
% subplot(2,3,2)
% plot(time_stamp_s,jointpos_x(:,limb_select), 'Linewidth', 2, 'color', [0 0.4470 0.7410])
% xlabel('Simulation Time(s)')
% ylabel('Joint Postion (mm)')
% title('End-Effector X Position from Baby Center')
% 
% subplot(2,3,3)
% plot(time_stamp_s,jointpos_y(:,limb_select), 'Linewidth', 2, 'color', [0 0.4470 0.7410])
% xlabel('Simulation Time(s)')
% ylabel('Joint Postion (mm)')
% title('End-Effector Y Position from Baby Center')
% 
% subplot(2,3,4)
% plot(time_stamp_s,cop_mag_mat_downsamp, 'Linewidth', 2, 'color', [0.6350 0.0780 0.1840])
% xlabel('Simulation Time(s)')
% ylabel('CoP Magnitude (mm)')
% title('CoP Magnitude from Force Mat')
% 
% subplot(2,3,5)
% plot(time_stamp_s,mat_x_downsamp, 'Linewidth', 2, 'color', [0.6350 0.0780 0.1840])
% xlabel('Simulation Time(s)')
% ylabel('CoP Magnitude (mm)')
% title('CoP X from Force Mat')
% 
% subplot(2,3,6)
% plot(time_stamp_s,mat_y_downsamp, 'Linewidth', 2, 'color', [0.6350 0.0780 0.1840])
% xlabel('Simulation Time(s)')
% ylabel('CoP Magnitude (mm)')
% title('CoP Y from Force Mat')
% 
% suptitle("Time series data from Simulator and Mat  "+"Limb: "+limb_name{limb_select})
% 
% 
% %% plotting mat vs sim data
% 
% figure();
% subplot(1,3,1)
% plot(sim_angle_raw(:,limb_select),cop_mag_mat_downsamp, 'Linewidth', 1.5, 'color', [0 0.4470 0.7410])
% xlabel('Joint Angle (deg)')
% ylabel('CoP Magnitude (mm)')
% title('CoP Magnitude vs Joint Angle')
% 
% subplot(1,3,2)
% plot(jointpos_x(:,limb_select),mat_x_downsamp, 'Linewidth', 1.5, 'color', [0.8500 0.3250 0.0980])
% xlabel('Joint Position (mm)')
% ylabel('CoP Magnitude (mm)')
% title('CoP Magnitude X vs Joint Position X')
% 
% subplot(1,3,3)
% plot(jointpos_y(:,limb_select),mat_y_downsamp, 'Linewidth', 1.5, 'color', [0.4660 0.6740 0.1880])
% xlabel('Joint Position (mm)')
% ylabel('CoP Magnitude (mm)')
% title('CoP Magnitude Y vs Joint Position Y')
