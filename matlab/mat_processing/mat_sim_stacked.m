%% load csv
main_folder = '/Users/jalpanchal/drive/penn/robo599/simulator_media/';
date = {'0429/'};
limb = {'right_hand/', 'left_hand/','right_leg/', 'left_leg/'};
limb_name = {'Right hand', 'Left Hand', 'Right Leg', 'Left Leg'};
limb_cols = {'rgthnd','lfthnd', 'rgtleg', 'lftleg'};
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
    target_samples = round(trial_time_s*30);
    
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

%% Calcualting stack windows
x = table2array(sim_data_alltrials(:,[limb_cols{limb_select}]));
[peak_val,peak_loc] = findpeaks(x);
stack_sim = {};
n_before = (peak_loc(1)+5);
n_after = (peak_loc(1)+5);
win_size = n_before+n_after+1;
%%%%for first window
win_start = 1;
win_stop = peak_loc(1)+n_after;
stack_temp_ = sim_data_alltrials(win_start : win_stop,:);
%marking peak
n_rows = size(stack_temp_,1);
n_cols = size(stack_temp_,2); 
stack_temp_.peak_mrk = zeros(n_rows,1);
stack_temp_.peak_mrk(peak_loc(1)) = 1;
%adding nan to match win size
stack_temp2_ = stack_temp_;
stack_temp2_(1:win_size-n_rows,:) = array2table(nan(win_size-n_rows,n_cols+1));
stack_temp2_((win_size-n_rows+1):end,:) = [];
stack_temp3_ = [stack_temp2_;stack_temp_];
stack_sim = [stack_sim;{stack_temp3_}];

%for 2: n-1 windows
for p = 2:(size(peak_loc,1)-1)
    win_start = peak_loc(p)-n_before;
    win_stop = peak_loc(p)+n_after;
    stack_temp_ = sim_data_alltrials(win_start : win_stop,:);
    
    %marking peak
    n_rows = size(stack_temp_,1);
    stack_temp_.peak_mrk = zeros(n_rows,1);
    stack_temp_.peak_mrk(n_before+1) = 1;
    stack_sim = [stack_sim;{stack_temp_}];
end

%for last window
win_start = peak_loc(end)-n_before;
stack_temp_ = sim_data_alltrials(win_start : end,:);
%marking peak
n_rows = size(stack_temp_,1);
n_cols = size(stack_temp_,2); 
stack_temp_.peak_mrk = zeros(n_rows,1);
stack_temp_.peak_mrk(n_before+1) = 1;
%adding nan to match win size
stack_temp2_ = stack_temp_;
stack_temp2_(1:win_size-n_rows,:) = array2table(nan(win_size-n_rows,n_cols+1));
stack_temp2_((win_size-n_rows+1):end,:) = [];
stack_temp3_ = [stack_temp_;stack_temp2_];
stack_sim = [stack_sim;{stack_temp3_}];

%%
for p = 1:size(stack_sim,1)
    plot(stack_sim{p,1}.rgtleg)
    hold on
end
    
%% plotting time series and Mat vs Simulator
% figure();

subplot(3,3,1)
plot(time_stamp_s,sim_angle_raw(:,limb_select), 'Linewidth', 2, 'color', [0 0.4470 0.7410])
hold on
plot(pk_loc,pk_val ,'r.', 'Markersize',10);
plot(pk_loc,pk_val ,'ro', 'Markersize',10);
hold off
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




