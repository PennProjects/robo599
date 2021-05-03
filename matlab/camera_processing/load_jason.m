command = 'pwd';
[~,cmdout] = system(command)

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

    
    %%

pathg_to_video = "/Users/jalpanchal/drive/penn/robo599/simulator_media/"

%--video /Users/jalpanchal/drivepenn/robo599/simulator_media/0422/lfthnd/cam_raw/gp_0422_lfthnd_2.MP4 --model_pose COCO --part_candidates --write_video /Users/jalpanchal/drive/penn/robo599/simulator_media/0422/lfthnd/openpose_out/gp_pose_0422_lfthnd_2.MP4