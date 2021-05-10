%% load csv
%experiment details
path_to_media = '/Users/jalpanchal/drive/penn/robo599/simulator_media/';
date = {'0429/'};
limb = {'right_hand/', 'left_hand/','right_leg/', 'left_leg/'};
device = {'mat/','simulator/','camera_trim/', 'openpose_out/'};
limb_name = {'Right hand', 'Left Hand', 'Right Leg', 'Left Leg'};
limb_cols = {'rgthnd','lfthnd', 'rgtleg', 'lftleg'};

%1-RH, 2-LH, 3-RL, 4-LL
limb_select = 1;

trials = 3;
trial_numbers = [1,2;1,2;1,2;1,3];
%read mat data

%read sim data
for j = 1:4
    path = strcat(path_to_media,date{1},limb{j},device{2});
    files = dir(strcat(path,'*.csv'));
    outs = cell(numel(files),1);
    for i = 1:numel(files)  
        file_name = strcat(path,files(i).name);
        raw_sim{j,i} = readtable(file_name);
    end
end

sim_data_alltrials = array2table(zeros(1,6));
sim_data_alltrials.Properties.VariableNames = {'time_ms','rgthnd','lfthnd', 'rgtleg', 'lftleg','trial_num'};
trial_data = [];
for t = 1:2
    t_num = trial_numbers(limb_select, t);
    sim_data_raw = raw_sim{limb_select, t_num};

    
    %resampling to 30 Hz
    %total samples
    sim_data_raw.time_ms = sim_data_raw.time_ms - sim_data_raw.time_ms(1);
    trial_time_s = sim_data_raw.time_ms(end)/1e3;
    target_samples = round(trial_time_s*30);
    trial_data = [trial_data;[trial_time_s,target_samples]];
        
    %resample sim data
    sim_data_resamp = sim_data_raw;
    sim_samples = size(sim_data_raw,1);
    sim_data_resamp = array2table(resample(table2array(sim_data_raw),target_samples,sim_samples));   
    %sim data
    trial_number_ = t_num*ones(size(sim_data_resamp,1),1);
    sim_data_resamp.trial_num = trial_number_;
    sim_data_resamp.Properties.VariableNames = {'time_ms','rgthnd','lfthnd', 'rgtleg', 'lftleg','trial_num'};
    sim_data_alltrials = [sim_data_alltrials;sim_data_resamp];
end
%removing zero entry
sim_data_alltrials(1,:) = [];


%load json pose data
op_out_path = strcat(path_to_media,date{1},limb{limb_select},device{4});
folder_name = dir(strcat(op_out_path,'gp_*'));

body_points = {'x0', 'x1', 'x2', 'x3', 'x4', 'x5', 'x6', 'x7', 'x8', 'x9', 'x10', 'x11', 'x12', 'x13', 'x14', 'x15', 'x16', 'x17'};
pose_raw = array2table(zeros(1,6));

for k = 1:numel(folder_name)
    trial_name = str2num(folder_name(k).name(end-5:end-5))
    files = dir(strcat(op_out_path,folder_name(k).name,'/','*.json'));
    for i = 1:numel(files)
        file_name = strcat(op_out_path,folder_name(k).name,'/',files(i).name);
        
        %open and copy file content
        fid = fopen(file_name);
        raw = fread(fid,inf);
        str = char(raw');
        fclose(fid);
        json_data = jsondecode(str);
        
        for j = 1:18
            trial_num = trial_name;
            frame_num = str2num(file_name(end-18:end-15));
            joint_idx = j-1;
            val_size = size(json_data.part_candidates.(body_points{j}),1);
            if val_size
                joint_pos = json_data.part_candidates.(body_points{j})(end-2:end)';
            else
                joint_pos = zeros(1,3);
            end
            temp_ = cell2table([{trial_num,frame_num, joint_idx} ,num2cell(joint_pos)]);
            pose_raw = [pose_raw;temp_];
        end
    end
end
pose_raw.Properties.VariableNames = {'trial_num','frame_num', 'joint_idx', 'x', 'y', 'c'};
pose_raw(1,:) = [];
%%
%test plot of end effector
r_ankle_x = pose_raw.x(pose_raw.joint_idx ==10 );
r_ankle_y = pose_raw.y(pose_raw.joint_idx ==10);

figure();
subplot(2,1,1)
plot(r_ankle_x);
title("Ankle X position")

subplot(2,1,2)
plot(r_ankle_y);
title("Ankle Y position")
suptitle("Joint position in pixels")
% for i =1:5:size(r_ankle_x,1)
%     plot(r_ankle_x(i), r_ankle_y(i), 'o-')
%     hold on
% end