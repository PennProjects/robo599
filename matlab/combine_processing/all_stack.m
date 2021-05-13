%experiment details
path_to_media = '/Users/jalpanchal/drive/penn/robo599/simulator_media/';
date = {'0429/'};
limb = {'right_hand/', 'left_hand/','right_leg/', 'left_leg/'};
device = {'mat/','simulator/','camera_trim/', 'openpose_out/'};
limb_name = {'Right hand', 'Left Hand', 'Right Leg', 'Left Leg'};
limb_cols = {'rgthnd','lfthnd', 'rgtleg', 'lftleg'};

%1-RH, 2-LH, 3-RL, 4-LL
limb_select = 3;

trials = 3;
trial_numbers = [1,2;1,2;1,2;1,3];

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

%read mat data
for j = 1:4
    path = strcat(path_to_media,date{1},limb{j},device{1});
    files = dir(strcat(path,'*.csv'));
    outs = cell(numel(files),1);
    for i = 1:numel(files)
        file_name = strcat(path,files(i).name);
        raw_mat{j,i} = readtable(file_name);
    end
end

mat_data_alltrials = array2table(zeros(1,4));
sim_data_alltrials = array2table(zeros(1,6));
sim_data_alltrials.Properties.VariableNames = {'time_ms','rgthnd','lfthnd', 'rgtleg', 'lftleg','trial_num'};
load('0429_mat_trunkidx.mat')
trial_data = [];
for t = 1:2
    t_num = trial_numbers(limb_select, t);
    sim_data_raw = raw_sim{limb_select, t_num};
    mat_data_raw = raw_mat{limb_select, t_num};

    %truncating data to experiment
    start_idx = start_idx_mat(limb_select,t_num) ;
    end_idx = end_idx_mat(limb_select,t_num);
    mat_data_trunk  = mat_data_raw(start_idx:end_idx,5:7);
    
    %resampling to 30 Hz
    %total samples
    sim_data_raw.time_ms = sim_data_raw.time_ms - sim_data_raw.time_ms(1);
    trial_time_s = sim_data_raw.time_ms(end)/1e3;
    target_samples = round(trial_time_s*30);
    trial_data = [trial_data;[trial_time_s,target_samples]];
        
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
sim_data_alltrials(1,:) = [];
mat_data_alltrials(1,:) = [];

%aading frame number
n_rows = size(sim_data_alltrials,1);
sim_data_alltrials.frame_num = (1:n_rows)';
mat_data_alltrials.frame_num = (1:n_rows)';

%remaming columns
mat_data_alltrials.Properties.VariableNames = {'cop_x','cop_y','r', 'trial_num', 'frame_num'};

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
%% Translate to World coordinates
%load calibration data
load("/Users/jalpanchal/drive/penn/robo599/simulator_media/0513/calibration/camera_calibration.mat");
xy_pixel = table2array( pose_raw(:,["x","y"]));

xy_world = array2table(pointsToWorld(cameraParams, R, t, xy_pixel));

pose_world = pose_raw;
% pose_world.Properties.VariableNames = {'frame_num', 'joint_idx', 'x_cal', 'y_cal', 'c'};
pose_world.x= xy_world.Var1;
pose_world.y= xy_world.Var2;

%%mat pixel to world
mat_pix = [520,75;520,1007;1454,75;1454,1007];
center = [(mat_pix(2,:) + mat_pix(3,:)).'/2]';

mat_cen_world = pointsToWorld(cameraParams, R, t, center)

%% Transpose all points to joint 1  and matreference
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
pose_data_smoothen = pose_mat;

for j = 0:17
    joint_pos = table2array(pose_data_smoothen(pose_data_smoothen.joint_idx ==j, ["x","y"]));
    out_lin = filloutliers(joint_pos(:,1:2),'linear', 'movmedian', 25);
    out_filt = sgolayfilt(out_lin,4,41);
    
    filt_pos = array2table(out_filt);
    pose_data_smoothen(pose_data_smoothen.joint_idx ==j, ["x","y"]) = filt_pos;  
end

%mat data smoothening
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


% %% outliar det test
% sim_angle = table2array(sim_data_alltrials(:,limb_cols));
% jointpos_x = [];
% jointpos_y = [];
% 
% for i = 1:size(sim_angle,1)
%     rh = calc_rh_pos(sim_angle(i,1));
%     lh = calc_lh_pos(sim_angle(i,2));
%     rl = calc_rl_pos(sim_angle(i,3));
%     ll = calc_ll_pos(sim_angle(i,4));
%     
%     jointpos_curr_x_ = [rh(end,1),lh(end,1),rl(end,1),ll(end,1)];
%     jointpos_x = [jointpos_x; jointpos_curr_x_];
%     
%     jointpos_curr_y_ = [rh(end,2),lh(end,2),rl(end,2),ll(end,2)];
%     jointpos_y = [jointpos_y; jointpos_curr_y_];
% end
% 
% ee_idx = [4,7,10,13];
% ee_x = pose_mat.x(pose_raw.joint_idx ==ee_idx(limb_select) );
% ee_y = pose_mat.y(pose_raw.joint_idx ==ee_idx(limb_select));
% 
% ee_x_sm = pose_data_smoothen.x(pose_raw.joint_idx ==ee_idx(limb_select) );
% ee_y_sm = pose_data_smoothen.y(pose_raw.joint_idx ==ee_idx(limb_select));
% 
% 
% 
% subplot(3,2,1)
% plot(jointpos_x(:,limb_select));
% title("Sim EE X position")
% 
% subplot(3,2,2)
% plot(jointpos_y(:,limb_select));
% title("Sim EE Y position") 
% 
% subplot(3,2,3)
% plot(ee_x);
% title("Cam EE X position")
% 
% subplot(3,2,4)
% plot(ee_y);
% title("Cam EE Y position Smooth")
% 
% subplot(3,2,5)
% plot(ee_x_sm);
% title("Cam EE X position")
% 
% subplot(3,2,6)
% plot(ee_y_sm);
% title("Cam EE Y position Smooth")
% suptitle("EE position")
% 



%% Calculating CoP magnitude and pose ee distance
mat_data_smoothen.cop_mag = vecnorm([mat_data_smoothen.cop_x, mat_data_smoothen.cop_y]')';
pose_data_smoothen.pos_mag = vecnorm([pose_data_smoothen.x, pose_data_smoothen.y]')';

%% Calcualting stack windows
x = table2array(sim_data_alltrials(:,[limb_cols{limb_select}]));
[peak_val,peak_loc] = findpeaks(x);
stack_idx = [];
n_before = (peak_loc(1)-1);
n_after = (peak_loc(1)+0);
win_size = n_before+n_after+1;
%%%%for first window
win_start = 1;
win_stop = peak_loc(1)+n_after;

stack_temp_ = (win_start : win_stop);


%duplicating first value to match win size
n_cols = size(stack_temp_,2);
stack_rep = ones(1,win_size-n_cols);
stack_temp_ = [stack_rep,stack_temp_];
stack_idx = [stack_idx;stack_temp_];


%for 2: n-1 windows
for p = 2:(size(peak_loc,1)-1)
    win_start = peak_loc(p)-n_before;
    win_stop = peak_loc(p)+n_after;
    
    stack_temp_ = (win_start : win_stop);
    stack_idx = [stack_idx;stack_temp_];
end

%for last window
win_start = peak_loc(end)-n_before;
win_stop = size(sim_data_alltrials,1);

stack_temp_ = (win_start : win_stop);

%adding nan to match win size
n_cols = size(stack_temp_,2);
stack_rep =stack_temp_(end)*ones(1,win_size-n_cols);
stack_temp_ = [stack_temp_,stack_rep];
stack_idx = [stack_idx;stack_temp_];


%% Calculating stack data

sim_angle_stack = [];
sim_x_stack = [];
sim_y_stack = [];
sim_z_stack = [];
sim_posmag_stack = [];

mat_copx_stack = [];
mat_copy_stack = [];
mat_copmag_stack = [];

pose_x_stack = [];
pose_y_stack = [];
pose_posmag_stack = [];

for s = 1:size(stack_idx,1)
    st_idx = stack_idx(s,:)';
    
    %sim data
    sim_angle = table2array(sim_data_alltrials(st_idx,limb_cols));
    jointpos_x = [];
    jointpos_y = [];
    jointpos_z = [];
    
    for i = 1:size(sim_angle,1)
        rh = calc_rh_pos(sim_angle(i,1));
        lh = calc_lh_pos(sim_angle(i,2));
        rl = calc_rl_pos(sim_angle(i,3));
        ll = calc_ll_pos(sim_angle(i,4));
        
        jointpos_curr_x_ = [rh(end,1),lh(end,1),rl(end,1),ll(end,1)];
        jointpos_x = [jointpos_x; jointpos_curr_x_];
        
        jointpos_curr_y_ = [rh(end,2),lh(end,2),rl(end,2),ll(end,2)];
        jointpos_y = [jointpos_y; jointpos_curr_y_];
        
        jointpos_curr_z_ = [rh(end,3),lh(end,3),rl(end,3),ll(end,3)];
        jointpos_z = [jointpos_z; jointpos_curr_z_];
    end
    
    sim_pos_mag = vecnorm([jointpos_x(:,limb_select), jointpos_y(:,limb_select)]')';
    
    sim_angle_stack  = [sim_angle_stack;sim_angle(:,limb_select)'];
    sim_x_stack = [sim_x_stack;jointpos_x(:,limb_select)'];
    sim_y_stack = [sim_y_stack;jointpos_y(:,limb_select)'];
    sim_z_stack = [sim_z_stack;jointpos_z(:,limb_select)'];
    sim_posmag_stack = [sim_posmag_stack;sim_pos_mag'];
    
    %mat data
    mat_copx_stack = [mat_copx_stack;table2array(mat_data_smoothen(st_idx,["cop_x"]))'];
    mat_copy_stack = [mat_copy_stack;table2array(mat_data_smoothen(st_idx,["cop_y"]))'];
    mat_copmag_stack = [mat_copmag_stack;table2array(mat_data_smoothen(st_idx,["cop_mag"]))'];
    
    
    %pose data
    ee_idx = [4,7,10,13];
    temp_ = [];
    for i = 1:size(stack_idx,2)
        idx = stack_idx(s,i);
        temp_ = [temp_;pose_data_smoothen(pose_data_smoothen.frame_num==idx...
                    & pose_data_smoothen.joint_idx==ee_idx(limb_select),:)];
    end
    pose_x_stack = [pose_x_stack;table2array(temp_(:,["x"]))'];
    pose_y_stack = [pose_y_stack;table2array(temp_(:,["y"]))'];
    pose_posmag_stack = [pose_posmag_stack;table2array(temp_(:,["pos_mag"]))'];
       
end


%% Calculating mean and std of stack

%sim data
sim_angle_stmean  = [sim_angle_stack;std(sim_angle_stack);...
                    mean(sim_angle_stack)];

sim_posx_stmean = [sim_x_stack;std(sim_x_stack);...
                  mean(sim_x_stack)];
sim_posy_stmean = [sim_y_stack;std(sim_y_stack);mean(sim_y_stack)];
sim_posmag_stmean = [sim_posmag_stack;std(sim_posmag_stack);mean(sim_posmag_stack)];

%mat
mat_copx_stmean = [mat_copx_stack;std(mat_copx_stack);...
                    mean(mat_copx_stack)];
mat_copy_stmean = [mat_copy_stack;std(mat_copy_stack);...
                    mean(mat_copy_stack)];
mat_copmag_stmean = [mat_copmag_stack;std(mat_copmag_stack);...
                    mean(mat_copmag_stack)];

%pose
pose_x_stmean = [pose_x_stack;std(pose_x_stack);...
                    mean(pose_x_stack)];
pose_y_stmean = [pose_y_stack;std(pose_y_stack);...
                    mean(pose_y_stack)];
pose_posmag_stmean = [pose_posmag_stack;std(pose_posmag_stack);...
                    mean(pose_posmag_stack)];


%%
figure(); 
nrSamples = 100; 
cMap = lines(nrSamples);


subplot(3,3,1)
[~] = stdshade(sim_posmag_stack,0.5,cMap(1,:)); 
grid on
xlabel('Frame number')
ylabel('Position Magnitude(mm)')
title('Simulator End-effector Position Magnitude')

subplot(3,3,2)
[~] = stdshade(sim_x_stack,0.5,cMap(1,:)); 
grid on
xlabel('Frame number')
ylabel('Position X (mm)')
title('Simulator End-effector  X position')

subplot(3,3,3)
[~] = stdshade(sim_y_stack,0.5,cMap(1,:)); 
grid on
xlabel('Frame number')
ylabel('Position Y (mm)')
title('Simulator End-effector  Y position')

subplot(3,3,4)
[~] = stdshade(mat_copmag_stack,0.5,cMap(3,:)); 
grid on
xlabel('Frame number')
ylabel('Position Magnitude (mm)')
title('Force Mat CoP Magnitude')

subplot(3,3,5)
[~] = stdshade(mat_copx_stack,0.5,cMap(3,:)); 
grid on
xlabel('Frame number')
ylabel('Position X (mm)')
title('Force Mat CoP X')

subplot(3,3,6)
[~] = stdshade(mat_copy_stack,0.5,cMap(3,:)); 
grid on
xlabel('Frame number')
ylabel('Position Y (mm)')
title('Force Mat CoP Y')


subplot(3,3,7)
[~] = stdshade(pose_posmag_stack,0.5,cMap(5,:)); 
grid on
xlabel('Frame number')
ylabel('Position Magnitude (mm)')
title('Camera End-Effector Position Magnitude')

subplot(3,3,8)
[~] = stdshade(pose_x_stack,0.5,cMap(5,:)); 
grid on
xlabel('Frame number')
ylabel('Position X (mm)')
title('Camera End-Effector X Position')

subplot(3,3,9)
[~] = stdshade(pose_y_stack,0.5,cMap(5,:)); 
grid on
xlabel('Frame number')
ylabel('Position Y (mm)')
title('Camera End-Effector Y Position')

suptitle("Comparison of Simultor, Mat and Cam data  "+"Limb: "+limb_name{limb_select})

%% Calculating correlations

r_sim_mat_mag = min(corrcoef(reshape(sim_posmag_stack', [],1),reshape(mat_copmag_stack', [],1)));
r_sim_mat_mag = round(r_sim_mat_mag(1),2);
r_sim_mat_x = min(corrcoef(reshape(sim_x_stack', [],1),reshape(mat_copx_stack', [],1)));
r_sim_mat_x = round(r_sim_mat_x(1),2);
r_sim_mat_x(isnan(r_sim_mat_x))=0;
r_sim_mat_y = min(corrcoef(reshape(sim_y_stack', [],1),reshape(mat_copy_stack', [],1)));
r_sim_mat_y = round(r_sim_mat_y(1),2);

r_pose_mat_mag = min(corrcoef(reshape(pose_posmag_stack', [],1),reshape(mat_copmag_stack', [],1)));
r_pose_mat_mag = round(r_pose_mat_mag(1),2);
r_pose_mat_x = min(corrcoef(reshape(pose_x_stack', [],1),reshape(mat_copx_stack', [],1)));
r_pose_mat_x = round(r_pose_mat_x(1),2);
r_pose_mat_x(isnan(r_pose_mat_x))=0;
r_pose_mat_y = min(corrcoef(reshape(pose_y_stack', [],1),reshape(mat_copy_stack', [],1)));
r_pose_mat_y = round(r_pose_mat_y(1),2);

r_sim_pose_mag = min(corrcoef(reshape(sim_posmag_stack', [],1),reshape(pose_posmag_stack', [],1)));
r_sim_pose_mag = round(r_sim_pose_mag(1),2);
r_sim_pose_x = min(corrcoef(reshape(sim_x_stack', [],1),reshape(pose_x_stack', [],1)));
r_sim_pose_x = round(r_sim_pose_x(1),2);
r_sim_pose_x(isnan(r_sim_pose_x))=0;
r_sim_pose_y = min(corrcoef(reshape(sim_y_stack', [],1),reshape(pose_y_stack', [],1)));
r_sim_pose_y = round(r_sim_pose_y(1),2);





%%

subplot(3,3,1)
plot(sim_posmag_stmean(end,1:peak_loc(1)),mat_copmag_stmean(end,1:peak_loc(1)), 'color', cMap(1,:), 'Linewidth', 2)
hold on
plot(sim_posmag_stmean(end,peak_loc(1):end),mat_copmag_stmean(end,peak_loc(1):end), 'color', cMap(2,:), 'Linewidth', 2)
plot(sim_posmag_stack(:,1:peak_loc(1)),mat_copmag_stack(:,1:peak_loc(1)), 'o','color', cMap(1,:))
plot(sim_posmag_stack(:,peak_loc(1):end),mat_copmag_stack(:,peak_loc(1):end), '^','color', cMap(2,:))
grid on
ylabel('Mat CoP Position Magnitude(mm)')
xlabel('Sim EE Position Magnitude(mm)')
legend("Flexion", "Extension")
title('Simulator EE vs Mat CoP : Position Magnitude' + "    r = "+ r_sim_mat_mag)

subplot(3,3,2)
plot(sim_posx_stmean(end,1:peak_loc(1)),mat_copx_stmean(end,1:peak_loc(1)), 'color', cMap(1,:), 'Linewidth', 2)

hold on
plot(sim_posx_stmean(end,peak_loc(1):end),mat_copx_stmean(end,peak_loc(1):end), 'color', cMap(2,:), 'Linewidth', 2)
plot(sim_x_stack(:,1:peak_loc(1)),mat_copx_stack(:,1:peak_loc(1)), 'o', 'color', cMap(1,:))
plot(sim_x_stack(:,peak_loc(1):end),mat_copx_stack(:,peak_loc(1):end), '^', 'color', cMap(2,:))
grid on
ylabel('Mat CoP Position X(mm)')
xlabel('Sim EE Position X(mm)')
legend("Flexion", "Extension")
title('Simulator EE vs Mat CoP : Position X' + "    r = "+ r_sim_mat_x)


subplot(3,3,3)
plot(sim_posy_stmean(end,1:peak_loc(1)),mat_copy_stmean(end,1:peak_loc(1)), 'color', cMap(1,:), 'Linewidth', 2)
hold on
plot(sim_posy_stmean(end,peak_loc(1):end),mat_copy_stmean(end,peak_loc(1):end), 'color', cMap(2,:), 'Linewidth', 2)
plot(sim_y_stack(:,1:peak_loc(1)),mat_copy_stack(:,1:peak_loc(1)), 'o', 'color', cMap(1,:))
plot(sim_y_stack(:,peak_loc(1):end),mat_copy_stack(:,peak_loc(1):end), '^', 'color', cMap(2,:))
grid on
ylabel('Mat CoP Position Y(mm)')
xlabel('Sim EE Position Y(mm)')
legend("Flexion", "Extension")
title('Simulator EE vs Mat CoP : Position Y' + "    r = "+ r_sim_mat_y)


subplot(3,3,4)
plot(pose_posmag_stmean(end,1:peak_loc(1)),mat_copmag_stmean(end,1:peak_loc(1)),'color', cMap(3,:), 'Linewidth', 2)

hold on
plot(pose_posmag_stmean(end,peak_loc(1):end),mat_copmag_stmean(end,peak_loc(1):end),'color', cMap(4,:), 'Linewidth', 2)
plot(pose_posmag_stack(:,1:peak_loc(1)),mat_copmag_stack(:,1:peak_loc(1)), 'o', 'color', cMap(3,:))
plot(pose_posmag_stack(:,peak_loc(1):end),mat_copmag_stack(:,peak_loc(1):end), '^', 'color', cMap(4,:))
grid on
ylabel('Mat CoP Position Magnitude(mm)')
xlabel('Cam EE Position Magnitude(mm)')
legend("Flexion", "Extension")
title('Camera EE vs Mat CoP : Position Magnitude' + "    r = "+ r_pose_mat_mag)


subplot(3,3,5)
plot(pose_x_stmean(end,1:peak_loc(1)),mat_copx_stmean(end,1:peak_loc(1)), 'color', cMap(3,:), 'Linewidth', 2)
hold on
plot(pose_x_stmean(end,peak_loc(1):end),mat_copx_stmean(end,peak_loc(1):end), 'color', cMap(4,:), 'Linewidth', 2)
plot(pose_x_stack(:,1:peak_loc(1)),mat_copx_stack(:,1:peak_loc(1)), 'o', 'color', cMap(3,:))
plot(pose_x_stack(:,peak_loc(1):end),mat_copx_stack(:,peak_loc(1):end), '^', 'color', cMap(4,:))
grid on
ylabel('Mat CoP Position X(mm)')
xlabel('Cam EE Position X(mm)')
legend("Flexion", "Extension")
title('Camera EE vs Mat CoP : Position X' + "    r = "+ r_pose_mat_x)

subplot(3,3,6)
plot(pose_y_stmean(end,1:peak_loc(1)),mat_copy_stmean(end,1:peak_loc(1)), 'color', cMap(3,:), 'Linewidth', 2)
hold on
plot(pose_y_stmean(end,peak_loc(1):end),mat_copy_stmean(end,peak_loc(1):end), 'color', cMap(4,:), 'Linewidth', 2)
plot(pose_y_stack(:,1:peak_loc(1)),mat_copy_stack(:,1:peak_loc(1)), 'o', 'color', cMap(3,:))
plot(pose_y_stack(:,peak_loc(1):end),mat_copy_stack(:,peak_loc(1):end), '^', 'color', cMap(4,:))
grid on
ylabel('Mat CoP Position Y(mm)')
xlabel('Cam EE Position Y(mm)')
legend("Flexion", "Extension")
title('Cam EE vs Mat CoP : Position Y' + "    r = "+ r_pose_mat_y)


subplot(3,3,7)
plot(sim_posmag_stmean(end,1:peak_loc(1)),pose_posmag_stmean(end,1:peak_loc(1)), 'color', cMap(5,:), 'Linewidth', 2)
hold on
plot(sim_posmag_stmean(end,peak_loc(1):end),pose_posmag_stmean(end,peak_loc(1):end), 'color', cMap(6,:), 'Linewidth', 2)
plot(sim_posmag_stack(:,1:peak_loc(1)),pose_posmag_stack(:,1:peak_loc(1)), 'o', 'color', cMap(5,:))
plot(sim_posmag_stack(:,peak_loc(1):end),pose_posmag_stack(:,peak_loc(1):end), '^', 'color', cMap(6,:))
grid on
ylabel('Cam EE Position Magnitude(mm)')
xlabel('Sim EE Position Magnitude(mm)')
legend("Flexion", "Extension")
title('Simulator EE vs Cam EE : Position Magnitude' + "    r = "+ r_sim_pose_mag)


subplot(3,3,8)
plot(sim_posx_stmean(end,1:peak_loc(1)),pose_x_stmean(end,1:peak_loc(1)), 'color', cMap(5,:), 'Linewidth', 2)
hold on
plot(sim_posx_stmean(end,peak_loc(1):end),pose_x_stmean(end,peak_loc(1):end), 'color', cMap(6,:), 'Linewidth', 2)
plot(sim_x_stack(:,1:peak_loc(1)),pose_x_stack(:,1:peak_loc(1)), 'o', 'color', cMap(5,:))
plot(sim_x_stack(:,peak_loc(1):end),pose_x_stack(:,peak_loc(1):end), '^', 'color', cMap(6,:))
grid on
ylabel('Cam EE Position X(mm)')
xlabel('Sim EE Position X(mm)')
legend("Flexion", "Extension")
title('Simulator EE vs Cam EE: Position X' + "    r = "+ r_sim_pose_x)


subplot(3,3,9)
plot(sim_posy_stmean(end,1:peak_loc(1)),pose_y_stmean(end,1:peak_loc(1)), 'color', cMap(5,:), 'Linewidth', 2)
hold on
plot(sim_posy_stmean(end,peak_loc(1):end),pose_y_stmean(end,peak_loc(1):end), 'color', cMap(6,:), 'Linewidth', 2)
plot(sim_y_stack(:,1:peak_loc(1)),pose_y_stack(:,1:peak_loc(1)), 'o', 'color', cMap(5,:))
plot(sim_y_stack(:,peak_loc(1):end),pose_y_stack(:,peak_loc(1):end), '^', 'color', cMap(6,:))
grid on
ylabel('Cam EE Position Y(mm)')
xlabel('Sim EE Position Y(mm)')
title('Simulator EE vs Cam EE: Position Y' + "    r = "+ r_sim_pose_y)
legend("Flexion", "Extension")
suptitle("Flexion and Extension trends "+"Limb: "+limb_name{limb_select})



%% Plotting Sim and Pose joints
baby_body = baby_body_points(); 
body_points = pose_data_smoothen;
ee_select = [4,7,10,13];
ee_pose_points = [];
ee_sim_points = [];
cop_mat_points = [];
com_points = [];
base_board = [-304.8,-304.8,-5;-304.8,304.8,-5;304.8,304.8,-5;304.8,-304.8,-5;-304.8,-304.8,-5];

for f = 1:size(stack_idx,2)

    frame_points = body_points(body_points.frame_num ==stack_idx(5,f),:);
    
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
        
   %collect ee points
    ee_pose_points = [ee_pose_points;[pose_x_stack(:,f),pose_y_stack(:,f)] ];
    
     ee_sim_points = [ee_sim_points;[sim_x_stack(:,f),sim_y_stack(:,f), sim_z_stack(:,f)] ];
     
     cop_mat_points = [cop_mat_points;[mat_copx_stack(:,f), mat_copy_stack(:,f)]];
     
     %sim data
    sim_angle = table2array(sim_data_alltrials(stack_idx(1,f),limb_cols));
    r_hand = calc_rh_pos(sim_angle(1,1));
    l_hand = calc_lh_pos(sim_angle(1,2));
    r_leg = calc_rl_pos(sim_angle(1,3));
    l_leg = calc_ll_pos(sim_angle(1,4));
    
    [com]=calc_com(sim_angle(1,1),sim_angle(1,2),sim_angle(1,3),sim_angle(1,4));
    [com_points] = [com_points;com];
    
    subplot(1,2,1);
    plot3(r_hand(:,1), r_hand(:,2), r_hand(:,3),'o-','LineWidth', 2,'color',cMap(1,:));
    hold on
    plot(ee_pose_points(:,1), ee_pose_points(:,2), 'o-', 'LineWidth', 2, 'color',cMap(2,:));
    plot(cop_mat_points(:,1), cop_mat_points(:,2), '^-', 'LineWidth', 2, 'color',cMap(3,:));
    plot3(com_points(:,1), com_points(:,2),com_points(:,3), '^-', 'LineWidth', 2', 'color',cMap(4,:));
    plot3(ee_sim_points(:,1), ee_sim_points(:,2),ee_sim_points(:,3) , 'o', 'LineWidth', 2, 'color',cMap(1,:));
    
    plot3(r_leg(:,1), r_leg(:,2), r_leg(:,3),'o-','LineWidth', 2,'color',cMap(1,:));
    plot3(l_hand(:,1), l_hand(:,2), l_hand(:,3),'o-','LineWidth', 2,'color',cMap(1,:));
    plot3(l_leg(:,1), l_leg(:,2), l_leg(:,3),'o-','LineWidth', 2,'color',cMap(1,:));
    plot3(baby_body(:,1), baby_body(:,2), baby_body(:,3),'o-','LineWidth', 2,'color','black');
    plot3(base_board(:,1),base_board(:,2),base_board(:,3),'o-','LineWidth', 2,'color','black'); 
    
    hold off
    grid on
    
    xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
    ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
    zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');
    legend("Simulator", "Camera", "CoP Mat", "Sim CoM", 'Location', 'northeast')
    title('Simulator angle')
    
    xlim([-350,350])
    ylim([-350,350])
    zlim([-30,200])
    view(0,90);
    
    
    subplot(1,2,2)
    plot(rh_points(:,1), rh_points(:,2), 'o-', 'LineWidth', 2,'color',cMap(2,:));
    hold on
    plot(ee_sim_points(:,1), ee_sim_points(:,2), 'o-', 'LineWidth', 2, 'color',cMap(1,:));
    plot(cop_mat_points(:,1), cop_mat_points(:,2), '^-', 'LineWidth', 2,'color',cMap(3,:));
    plot(com_points(:,1), com_points(:,2), '^-', 'LineWidth', 2, 'color',cMap(4,:));
    plot(ee_pose_points(:,1), ee_pose_points(:,2), 'o', 'LineWidth', 2, 'color',cMap(2,:));
    
    plot(lh_points(:,1), lh_points(:,2), 'o-','LineWidth', 2,'color',cMap(2,:));
    plot(rl_points(:,1), rl_points(:,2), 'o-','LineWidth', 2,'color',cMap(2,:));
    plot(ll_points(:,1), ll_points(:,2), 'o-','LineWidth', 2,'color',cMap(2,:));
    plot(base_board(:,1),base_board(:,2),'o-','LineWidth', 2,'color','black'); 
    
    hold off
    grid on 
    xlabel('Distance (mm)')
    ylabel('Distance (mm)')
    xlim([-350,350])
    ylim([-350,350])
   legend("Camera","Simulator", "CoP Mat", "Sim CoM")
    title("Pose Detection from Camera")
    drawnow
    
    
end

%%
plot3(r_hand(:,1), r_hand(:,2), r_hand(:,3),'o-','LineWidth', 2,'color',cMap(1,:));
hold on
plot(ee_pose_points(:,1), ee_pose_points(:,2), 'o-', 'LineWidth', 2, 'color',cMap(2,:));
plot(cop_mat_points(:,1), cop_mat_points(:,2), '^-', 'LineWidth', 2, 'color',cMap(3,:));
plot3(com_points(:,1), com_points(:,2),com_points(:,3), '^-', 'LineWidth', 2', 'color',cMap(4,:));
plot3(ee_sim_points(:,1), ee_sim_points(:,2),ee_sim_points(:,3) , 'o', 'LineWidth', 2, 'color',cMap(1,:));

plot3(r_leg(:,1), r_leg(:,2), r_leg(:,3),'o-','LineWidth', 2,'color',cMap(1,:));
plot3(l_hand(:,1), l_hand(:,2), l_hand(:,3),'o-','LineWidth', 2,'color',cMap(1,:));
plot3(l_leg(:,1), l_leg(:,2), l_leg(:,3),'o-','LineWidth', 2,'color',cMap(1,:));
plot3(baby_body(:,1), baby_body(:,2), baby_body(:,3),'o-','LineWidth', 2,'color','black');
plot3(base_board(:,1),base_board(:,2),base_board(:,3),'o-','LineWidth', 2,'color','black');

hold off
grid on

xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');
legend("Simulator", "Camera", "CoP Mat", "Sim CoM", 'Location', 'northeast')
title('Simulator angle')

xlim([-350,350])
ylim([-350,350])
zlim([-30,200])
view(0,90);




