%% load csv
%experiment details
path_to_media = '/Users/jalpanchal/drive/penn/robo599/simulator_media/';
date = {'0429/'};
limb = {'right_hand/', 'left_hand/','right_leg/', 'left_leg/'};
device = {'mat/','simulator/','camera_trim/', 'openpose_out/'};
limb_name = {'Right hand', 'Left Hand', 'Right Leg', 'Left Leg'};
limb_cols = {'rgthnd','lfthnd', 'rgtleg', 'lftleg'};

%1-RH, 2-LH, 3-RL, 4-LL
limb_select = 4;

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
%aading frame number
n_rows = size(sim_data_alltrials,1);
sim_data_alltrials.frame_num = (1:n_rows)';

%load json pose data
op_out_path = strcat(path_to_media,date{1},limb{limb_select},device{4});
folder_name = dir(strcat(op_out_path,'gp_*'));

body_points = {'x0', 'x1', 'x2', 'x3', 'x4', 'x5', 'x6', 'x7', 'x8', 'x9', 'x10', 'x11', 'x12', 'x13', 'x14', 'x15', 'x16', 'x17'};
pose_raw = array2table(zeros(1,6));

for k = 1:numel(folder_name)
    trial_num = str2num(folder_name(k).name(end-5:end-5))
    files = dir(strcat(op_out_path,folder_name(k).name,'/','*.json'));
    
    %trimming data to experiment
    start_trim_frames = [8,2;3,2;3,3;2,3];
    end_trim_frames = [7,5;6,7;5,6;3,7];
        
    for i = start_trim_frames(limb_select,k):numel(files)-end_trim_frames(limb_select,k)
        file_name = strcat(op_out_path,folder_name(k).name,'/',files(i).name);
        
        %open and copy file content
        fid = fopen(file_name);
        raw = fread(fid,inf);
        str = char(raw');
        fclose(fid);
        json_data = jsondecode(str);
        
        for j = 1:18
            frame_num = str2num(file_name(end-18:end-15));
            joint_idx = j-1;
            val_size = size(json_data.part_candidates.(body_points{j}),1);
            if val_size
                joint_pos = json_data.part_candidates.(body_points{j})(end-2:end)';
            else
                joint_pos = zeros(1,3);
            end
            temp_ = cell2table([{trial_num,frame_num, joint_idx} ,num2cell(joint_pos)]);
            temp_.Var2  = temp_.Var2-start_trim_frames(limb_select,k)+2; %restart frame number from 1
            pose_raw = [pose_raw;temp_];
        end
    end        
end
%renaming the frames from 1:n
pose_raw.Var2(pose_raw.Var1  > 1) = pose_raw.Var2(pose_raw.Var1  > 1)+ trial_data(1,2);
pose_raw.Properties.VariableNames = {'trial_num','frame_num', 'joint_idx', 'x', 'y', 'c'};
pose_raw(1,:) = [];


%% plot raw ee position
sim_angle = table2array(sim_data_alltrials(:,limb_cols));
jointpos_x = [];
jointpos_y = [];

for i = 1:size(sim_angle,1)
    rh = calc_rh_pos(sim_angle(i,1));
    lh = calc_lh_pos(sim_angle(i,2));
    rl = calc_rl_pos(sim_angle(i,3));
    ll = calc_ll_pos(sim_angle(i,4));
    
    jointpos_curr_x_ = [rh(end,1),lh(end,1),rl(end,1),ll(end,1)];
    jointpos_x = [jointpos_x; jointpos_curr_x_];
    
    jointpos_curr_y_ = [rh(end,2),lh(end,2),rl(end,2),ll(end,2)];
    jointpos_y = [jointpos_y; jointpos_curr_y_];
end

ee_idx = [4,7,10,13];
ee_x = pose_raw.x(pose_raw.joint_idx ==ee_idx(limb_select) );
ee_y = pose_raw.y(pose_raw.joint_idx ==ee_idx(limb_select));



subplot(2,2,1)
plot(jointpos_x(:,limb_select));
title("Sim EE X position")

subplot(2,2,2)
plot(jointpos_y(:,limb_select));
title("Sim EE Y position") 

subplot(2,2,3)
plot(ee_x);
title("Cam EE X position")

subplot(2,2,4)
plot(ee_y);
title("Cam EE Y position")
suptitle("EE position") 



%% Translate to World coordinates
%load calibration data
load("/Users/jalpanchal/drive/penn/robo599/simulator_media/0429/calibration/camera_calibration.mat");
xy_pixel = table2array( pose_raw(:,["x","y"]));

xy_world = array2table(pointsToWorld(cameraParams, R, t, xy_pixel));

pose_world = pose_raw;
% pose_world.Properties.VariableNames = {'frame_num', 'joint_idx', 'x_cal', 'y_cal', 'c'};
pose_world.x= xy_world.Var1;
pose_world.y= xy_world.Var2;

%%mat pixel to world
mat_pix = [520,75;520,1007;1454,75;1454,1007];
center = [(mat_pix(2,:) + mat_pix(3,:)).'/2]';

mat_cen_world = pointsToWorld(cameraParams, R, t, center);

%% Transpose all points to joint 1 and mat reference
pose_sim = pose_world;
pose_mat = pose_sim;
trial_numbers = unique(pose_sim.trial_num);
total_trials = size(trial_numbers,1);

for t = 1:total_trials
    t_num = trial_numbers(t);
    temp_ = pose_sim(pose_sim.trial_num==t_num,:);
    total_frames = temp_.frame_num(end,:);
    
    for f = 0:total_frames
        pose_sim.x(pose_sim.trial_num==t_num & pose_sim.frame_num ==f,:) = ...
            pose_sim.x(pose_sim.trial_num==t_num & pose_sim.frame_num ==f,:)-...
            pose_sim.x(pose_sim.trial_num==t_num & pose_sim.frame_num ==f & pose_sim.joint_idx == 1,:);
        pose_sim.y(pose_sim.trial_num==t_num & pose_sim.frame_num ==f,:) = -1*(...
            pose_sim.y(pose_sim.trial_num==t_num & pose_sim.frame_num ==f,:)-...
            pose_sim.y(pose_sim.trial_num==t_num & pose_sim.frame_num ==f & pose_sim.joint_idx == 1,:));
        
        %transpost to mat center
        pose_mat.x(pose_mat.trial_num==t_num & pose_mat.frame_num ==f,:) = ...
            pose_mat.x(pose_mat.trial_num==t_num & pose_mat.frame_num ==f,:)-...
            mat_cen_world(1);
        pose_mat.y(pose_mat.trial_num==t_num & pose_mat.frame_num ==f,:) = -1*(...
            pose_mat.y(pose_sim.trial_num==t_num & pose_mat.frame_num ==f,:)-...
            mat_cen_world(2));
    end
end

%% Correcting outliars and smoothening
%First we isolat eteh outliars and then smoothen all the joints witha 4th
%order SG filter
pose_filt = pose_mat;

for j = 1:17
    joint_pos = table2array(pose_filt(pose_filt.joint_idx ==j, ["x","y"]));
    out_lin = filloutliers(joint_pos(:,1:2),'linear', 'movmedian', 75);
% %     out_lin = joint_pos;
    out_filt = sgolayfilt(out_lin,4,41);
%     out_filt = out_lin;

%     out_lin = filloutliers(joint_pos(:,1:2),'linear');
%     out_filt = sgolayfilt(out_lin,4,19);
    
    filt_pos = array2table(out_filt);
    pose_filt(pose_filt.joint_idx ==j, ["x","y"]) = filt_pos;  
end

sim_angle = table2array(sim_data_alltrials(:,limb_cols));
jointpos_x = [];
jointpos_y = [];

for i = 1:size(sim_angle,1)
    rh = calc_rh_pos(sim_angle(i,1));
    lh = calc_lh_pos(sim_angle(i,2));
    rl = calc_rl_pos(sim_angle(i,3));
    ll = calc_ll_pos(sim_angle(i,4));
    
    jointpos_curr_x_ = [rh(end,1),lh(end,1),rl(end,1),ll(end,1)];
    jointpos_x = [jointpos_x; jointpos_curr_x_];
    
    jointpos_curr_y_ = [rh(end,2),lh(end,2),rl(end,2),ll(end,2)];
    jointpos_y = [jointpos_y; jointpos_curr_y_];
end

ee_idx = [4,7,10,13];
ee_x = pose_mat.x(pose_raw.joint_idx ==ee_idx(limb_select) );
ee_y = pose_mat.y(pose_raw.joint_idx ==ee_idx(limb_select));

ee_x_sm = pose_filt.x(pose_raw.joint_idx ==ee_idx(limb_select) );
ee_y_sm = pose_filt.y(pose_raw.joint_idx ==ee_idx(limb_select));



subplot(3,2,1)
plot(jointpos_x(:,limb_select));
title("Sim EE X position")

subplot(3,2,2)
plot(jointpos_y(:,limb_select));
title("Sim EE Y position") 

subplot(3,2,3)
plot(ee_x);
title("Cam EE X position")

subplot(3,2,4)
plot(ee_y);
title("Cam EE Y position Smooth")

subplot(3,2,5)
plot(ee_x_sm);
title("Cam EE X position")

subplot(3,2,6)
plot(ee_y_sm);
title("Cam EE Y position Smooth")
suptitle("EE position")


%% Calcualting stack windows
x = table2array(sim_data_alltrials(:,[limb_cols{limb_select}]));
[peak_val,peak_loc] = findpeaks(x);
sim_stack = {};
stack_idx = [];
n_before = (peak_loc(1)-1);
n_after = (peak_loc(1)+0);
win_size = n_before+n_after+1;
%%%%for first window
win_start = 1;
win_stop = peak_loc(1)+n_after;

sim_stack_temp_ = sim_data_alltrials(win_start : win_stop,:);
stack_temp_ = (win_start : win_stop);

%marking peak
n_rows = size(sim_stack_temp_,1);
n_cols_sim = size(sim_stack_temp_,2); 
sim_stack_temp_.peak_mrk = zeros(n_rows,1);


%adding nan to match win size
sim_stack_temp2_ = sim_stack_temp_;
sim_stack_temp2_(1:win_size-n_rows,:) = array2table(nan(win_size-n_rows,n_cols_sim+1));
sim_stack_temp2_((win_size-n_rows+1):end,:) = [];
sim_stack_temp3_ = [sim_stack_temp2_;sim_stack_temp_];
sim_stack = [sim_stack;{sim_stack_temp3_}];

stack_nan = nan(1,win_size-n_rows);
stack_temp_ = [stack_nan,stack_temp_];
stack_idx = [stack_idx;stack_temp_];


%for 2: n-1 windows
for p = 2:(size(peak_loc,1)-1)
    win_start = peak_loc(p)-n_before;
    win_stop = peak_loc(p)+n_after;
    
    sim_stack_temp_ = sim_data_alltrials(win_start : win_stop,:);
    stack_temp_ = (win_start : win_stop);
    
    %marking peak
    n_rows = size(sim_stack_temp_,1);
    sim_stack_temp_.peak_mrk = zeros(n_rows,1);
    sim_stack_temp_.peak_mrk(n_before+1) = 1;
    
    
    sim_stack = [sim_stack;{sim_stack_temp_}];
    stack_idx = [stack_idx;stack_temp_];
end

%for last window
win_start = peak_loc(end)-n_before;
win_stop = size(sim_data_alltrials,1);

sim_stack_temp_ = sim_data_alltrials(win_start : win_stop,:);
stack_temp_ = (win_start : win_stop);

%marking peak
n_rows = size(sim_stack_temp_,1);
n_cols_sim = size(sim_stack_temp_,2); 
sim_stack_temp_.peak_mrk = zeros(n_rows,1);
sim_stack_temp_.peak_mrk(n_before+1) = 1;
%adding nan to match win size

sim_stack_temp2_ = sim_stack_temp_;
sim_stack_temp2_(1:win_size-n_rows,:) = array2table(nan(win_size-n_rows,n_cols_sim+1));
sim_stack_temp2_((win_size-n_rows+1):end,:) = [];
sim_stack_temp3_ = [sim_stack_temp_;sim_stack_temp2_];
sim_stack = [sim_stack;{sim_stack_temp3_}];

stack_nan = nan(1,win_size-n_rows);
stack_temp_ = [stack_temp_,stack_nan];
stack_idx = [stack_idx;stack_temp_];


%% test stack

% for p = 1:size(sim_stack,1)
%     subplot(1,2,1)
%     plot(sim_stack{p,1}.rgtleg)
%     hold on
% end
% 
% for s = 1:size(stack_idx,1)
%     st_idx = stack_idx(s,:)';
%     
%     ang = table2array(sim_data_alltrials(st_idx,["rgtleg"]));
%     subplot(1,2,2)
%     plot(ang);
%     hold on
% end


% for s = 1:size(stack_idx,1)
%     ang = [];
%     for i = 1:size(stack_idx,2)
%         idx = stack_idx(s,i);
%         
%         ang = [ang;pose_filt.x(pose_filt.frame_num==idx & pose_filt.joint_idx==10,:)];
%         
%     end
%     plot(ang)
%     hold on
% end


%%
body_points = pose_filt;

total_frames = body_points.frame_num(end);
figure();
for f = 1:total_frames
    frame_points = body_points(body_points.frame_num ==f,:);
    
    %right hand = 1,2,3,4
    rh_points = table2array([frame_points(frame_points.joint_idx==1,["x","y"]);
                 frame_points(frame_points.joint_idx==2,["x","y"]);
                 frame_points(frame_points.joint_idx==3,["x","y"]);
                 frame_points(frame_points.joint_idx==4,["x","y"])]);
    %left hand = 1,5,6,7
    lh_points = table2array([frame_points(frame_points.joint_idx==1,["x","y"]);
                 frame_points(frame_points.joint_idx==5,["x","y"]);
                 frame_points(frame_points.joint_idx==6,["x","y"]);
                 frame_points(frame_points.joint_idx==7,["x","y"])]);
    %right leg = 1,8,9,10
    rl_points = table2array([frame_points(frame_points.joint_idx==1,["x","y"]);
                 frame_points(frame_points.joint_idx==8,["x","y"]);
                 frame_points(frame_points.joint_idx==9,["x","y"]);
                 frame_points(frame_points.joint_idx==10,["x","y"])]);
    %left leg  = 1,11,12,13
    ll_points = table2array([frame_points(frame_points.joint_idx==1,["x","y"]);
                 frame_points(frame_points.joint_idx==11,["x","y"]);
                 frame_points(frame_points.joint_idx==12,["x","y"]);
                 frame_points(frame_points.joint_idx==13,["x","y"])]);
%     rh_points = table2array(rh_points);
%     lh_points = table2array(lh_points);
    
    plot(rh_points(:,1), rh_points(:,2), 'o-', 'LineWidth', 2,'color','r');
    hold on
    plot(lh_points(:,1), lh_points(:,2), 'o-','LineWidth', 2,'color','b');
    plot(rl_points(:,1), rl_points(:,2), 'o-','LineWidth', 2,'color','r');
    plot(ll_points(:,1), ll_points(:,2), 'o-','LineWidth', 2,'color','b');
    hold off
    grid on 
    xlabel('Distance (mm)')
    ylabel('Distance (mm)')
    xlim([-250,250])
    ylim([-300,300])
    title("Body Joints Calibrated, Sim ref and smoothened")
    drawnow
end 
