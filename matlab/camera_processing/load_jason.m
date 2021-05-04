%% Generate Openpose Command
%navigate to openpose
command = "cd /Users/jalpanchal/git/openpose/";
system(command);

path_to_op = "./build/examples/openpose/openpose.bin ";
input_param = "--video ";
model_param = " --model_pose COCO --part_candidates ";
output_param_video ="--write_video ";
output_param_json = "--write_json ";

%experiment details
path_to_media = '/Users/jalpanchal/drive/penn/robo599/simulator_media/';
date = {'0429/'};
limb = {'right_hand/', 'left_hand/','right_leg/', 'left_leg/'};
device = {'camera_trim/', 'openpose_out/'};

%gen
limb_idx = 1;
vid_in_path = strcat(path_to_media,date{1},limb{limb_idx},device{1});
op_out_path = strcat(path_to_media,date{1},limb{limb_idx},device{2});
files = dir(strcat(vid_in_path,'*.mp4'));
for i = 1:numel(files)
    video_file = strcat(vid_in_path,files(i).name);
    out_file = strcat(op_out_path,"out_",files(i).name);
    
    fprintf(limb{limb_idx})
    fprintf("Command for Experiment : %d",i);
    %command for video out
    video_out_cmd = strcat(path_to_op,input_param,video_file,model_param,output_param_video,out_file)  
    
    %%command for json out
    json_out_cmd = strcat(path_to_op,input_param,video_file,model_param,output_param_json,op_out_path)
end

%% Open json files
op_out_path = strcat(path_to_media,date{1},limb{3},device{2});
files = dir(strcat(op_out_path,'*.json'));

body_points = {'x0', 'x1', 'x2', 'x3', 'x4', 'x5', 'x6', 'x7', 'x8', 'x9', 'x10', 'x11', 'x12', 'x13', 'x14', 'x15', 'x16', 'x17'};
pose_raw = array2table(zeros(1,5));
for i = 1:numel(files)
   file_name = strcat(op_out_path,files(i).name);
   
   %open and copy file content
   fid = fopen(file_name); 
   raw = fread(fid,inf);
   str = char(raw');
   fclose(fid);
   json_data = jsondecode(str);
   
   for j = 1:18
       frame_num = str2num(file_name(end-18:end-15));
       joint_idx = j-1;
       joint_pos = json_data.part_candidates.(body_points{j})(end-2:end)';
       temp_ = array2table([frame_num, joint_idx, joint_pos]);
       pose_raw = [pose_raw;temp_];
   end   
end
pose_raw.Properties.VariableNames = {'frame_num', 'joint_idx', 'x', 'y', 'c'};

%%
%test plot of wrist
r_ankle_x = pose_raw.x(pose_raw.joint_idx ==10);
r_ankle_y = pose_raw.y(pose_raw.joint_idx ==10);

figure();
subplot(2,1,1)
plot(r_ankle_x);

subplot(2,1,2)
plot(r_ankle_y);
% for i =1:5:size(r_ankle_x,1)
%     plot(r_ankle_x(i), r_ankle_y(i), 'o-')
%     hold on
% end

%%
a = pose_raw(pose_raw.joint_idx ==10,:);

