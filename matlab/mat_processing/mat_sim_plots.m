%% load csv
main_folder = '/Users/jalpanchal/drive/penn/robo599/simulator_media/';
date = {'0429/'};
limb = {'right_hand/', 'left_hand/','right_leg/', 'left_leg/'};
device = {'mat/','simulator/'};
trials = 3;

%read mat data
for j = 1:4
    path = strcat(main_folder,date{1},limb{j},device{1});
    files = dir(strcat(path,'*.csv'));
    outs = cell(numel(files),1);
    for i = 1:numel(files)  % Could be parfor
        file_name = strcat(path,files(i).name);
        raw_mat{j,i} = readtable(file_name);
    end
end

%read sim data
for j = 1:4
    path = strcat(main_folder,date{1},limb{j},device{2});
    files = dir(strcat(path,'*.csv'));
    outs = cell(numel(files),1);
    for i = 1:numel(files)  % Could be parfor
        file_name = strcat(path,files(i).name);
        raw_sim{j,i} = readtable(file_name);
    end
end
%%
%1-RH, 2-LH, 3-RL, 4-LL
limb_select = 3;
exp_num = 1;

mat_data_raw = raw_mat{limb_select, exp_num};
sim_data_raw = raw_sim{limb_select, exp_num};


%% Calculating COP magnitude magnitude

mat_x_raw= mat_data_raw.Var5;
mat_y_raw= mat_data_raw.Var6;
cop_mag_mat = vecnorm([mat_x_raw, mat_y_raw]')';

%% truncating data to experiment
start_idx_mat = [180,95 0; 100 80 0; 120 110 0; 220 0 100];
end_idx_mat = [934, 851 0; 885 860 0; 811 803 0; 970 0 740 ];
start_idx = start_idx_mat(limb_select,exp_num) ;
end_idx = end_idx_mat(limb_select,exp_num);
mat_data_trunk  = mat_data_raw(start_idx:end_idx,:);

mat_x_trunk= mat_data_trunk.Var5;
mat_y_trunk= mat_data_trunk.Var6;
copmag_mat_trunk = vecnorm([mat_x_trunk, mat_y_trunk]')';

%% Down sampling mat data to match sim data
mat_data_downsamp  = mat_data_trunk;
sim_data = sim_data_raw;
mat_data_size = size(mat_data_downsamp,1);
sim_data_size = size(sim_data,1);
mat_x_downsamp = resample(mat_data_downsamp.Var5,sim_data_size,mat_data_size);
mat_y_downsamp = resample(mat_data_downsamp.Var6, sim_data_size, mat_data_size);

cop_mag_mat_downsamp = vecnorm([mat_x_downsamp, mat_y_downsamp]')';
%% fetching Joint positions
sim_angle_raw  = sim_data_raw(:,1+limb_select).Variables;
plot(sim_angle_raw)


%% plotting time series

sim_angle_raw  = sim_data_raw(:,1+limb_select);

subplot(2,3,1)
dataset  = mat_data_trunk;
plot(dataset.Var5, dataset.Var6, 'o-')
xlim([-20,5])
ylim([-10,30])

subplot(2,2,2)
plot(cop_mag_mat)

subplot(2,2,3)
plot(mat_x_downsamp, mat_y_downsamp, 'o-')
xlim([-20,5])
ylim([-10,30])

subplot(2,2,4)
plot(cop_mag_mat_downsamp)
plot(sim_angle_raw.Variables,cop_mag_mat_downsamp)
xlabel('Sim Angle(deg)')
ylabel('CoP magnitude (mm)')
title("CoP magnitude vs limb angle "+" Limb: "+limb_select+" Exp: "+exp_num)

%% Impact of down sampling
subplot(2,2,1)
dataset  = mat_data_trunk;
plot(dataset.Var5, dataset.Var6, 'o-')
xlim([-20,5])
ylim([-10,30])

subplot(2,2,2)
plot(cop_mag_mat)

subplot(2,2,3)
plot(mat_x_downsamp, mat_y_downsamp, 'o-')
xlim([-20,5])
ylim([-10,30])

subplot(2,2,4)
plot(cop_mag_mat_downsamp)

%%