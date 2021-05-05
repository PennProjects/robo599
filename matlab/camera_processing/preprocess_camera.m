%experiment details
path_to_media = '/Users/jalpanchal/drive/penn/robo599/simulator_media/';
date = {'0429/'};
limb = {'right_hand/', 'left_hand/','right_leg/', 'left_leg/'};
device = {'camera_trim/', 'openpose_out/'};
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
pose_raw(1,:) = [];

%%
%test plot of wrist
r_ankle_x = pose_raw.x(pose_raw.joint_idx ==10);
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

%% Translate to World coordinates
%load calibration data
load("/Users/jalpanchal/drive/penn/robo599/simulator_media/0429/calibration/camera_calibration.mat");
xy_pixel = table2array( pose_raw(:,3:4));

xy_world = array2table(pointsToWorld(cameraParams, R, t, xy_pixel));

pose_world = pose_raw;
% pose_world.Properties.VariableNames = {'frame_num', 'joint_idx', 'x_cal', 'y_cal', 'c'};
pose_world.x= xy_world.Var1;
pose_world.y= xy_world.Var2;

%%
%test plot of wrist
r_ankle_x = pose_world.x(pose_world.joint_idx ==10);
r_ankle_y = pose_world.y(pose_world.joint_idx ==10);

figure();
subplot(2,1,1)
plot(r_ankle_x);
title("Ankle X position")

subplot(2,1,2)
plot(r_ankle_y);
title("Ankle Y position")
suptitle("Joint position in World")


%% Transpose all points to joint 1 reference
pose_sim = pose_world;
total_frames = pose_sim.frame_num(end);
for f = 0:total_frames
    pose_sim.x(pose_sim.frame_num ==f,:) = pose_sim.x(pose_sim.frame_num ==f,:)-pose_sim.x(pose_sim.frame_num ==f & pose_sim.joint_idx == 1,:);
    pose_sim.y(pose_sim.frame_num ==f,:) = pose_sim.y(pose_sim.frame_num ==f,:)-pose_sim.y(pose_sim.frame_num ==f & pose_sim.joint_idx == 1,:);    
end


%%
%Correcting outliars and smoothening
%test
joint_idx = 9;


joint_pos = table2array(pose_sim(pose_sim.joint_idx ==joint_idx, 3:5));
subplot(2,1,1)
plot(joint_pos(:,1:2))

out_lin = filloutliers(joint_pos(:,1:2),'linear');
% subplot(2,1,2)
% plot(out_lin)

out_filt = sgolayfilt(out_lin,4,19);
subplot(2,1,2)
plot(out_filt)


%% Correcting outliars and smoothening
%First we isolat eteh outliars and then smoothen all the joints witha 4th
%order SG filter
pose_filt = pose_sim;

for j = 1:17
    joint_pos = table2array(pose_filt(pose_filt.joint_idx ==j, 3:4));
    out_lin = filloutliers(joint_pos(:,1:2),'linear');
    out_filt = sgolayfilt(out_lin,4,19);
    
    filt_pos = array2table(out_filt);
    pose_filt(pose_filt.joint_idx ==j, 3:4) = filt_pos;  
end



%% plot body joints
body_points = pose_raw;

total_frames = body_points.frame_num(end);
% figure();
for f = 0:total_frames
    frame_points = body_points(body_points.frame_num ==f,:);
    
    %right hand = 1,2,3,4
    rh_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==2,3:4);
                 frame_points(frame_points.joint_idx==3,3:4);
                 frame_points(frame_points.joint_idx==4,3:4)]);
    %left hand = 1,5,6,7
    lh_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==5,3:4);
                 frame_points(frame_points.joint_idx==6,3:4);
                 frame_points(frame_points.joint_idx==7,3:4)]);
    %right leg = 1,8,9,10
    rl_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==8,3:4);
                 frame_points(frame_points.joint_idx==9,3:4);
                 frame_points(frame_points.joint_idx==10,3:4)]);
    %left leg  = 1,11,12,13
    ll_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==11,3:4);
                 frame_points(frame_points.joint_idx==12,3:4);
                 frame_points(frame_points.joint_idx==13,3:4)]);
%     rh_points = table2array(rh_points);
%     lh_points = table2array(lh_points);
    
    subplot(2,2,1)
    plot(rh_points(:,1), rh_points(:,2), 'o-', 'LineWidth', 2,'color','r');
    set(gca, 'YDir','reverse')
    hold on
    plot(lh_points(:,1), lh_points(:,2), 'o-','LineWidth', 2,'color','b');
    plot(rl_points(:,1), rl_points(:,2), 'o-','LineWidth', 2,'color','r');
    plot(ll_points(:,1), ll_points(:,2), 'o-','LineWidth', 2,'color','b');
    hold off
    grid on 
    xlabel('Distance (mm)')
    ylabel('Distance (mm)')
    xlim([-500,500])
    ylim([-100, 540])
    title("Body Joints Raw From OpenPose")


body_points = pose_filt;

total_frames = body_points.frame_num(end);
% figure();

    frame_points = body_points(body_points.frame_num ==f,:);
    
    %right hand = 1,2,3,4
    rh_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==2,3:4);
                 frame_points(frame_points.joint_idx==3,3:4);
                 frame_points(frame_points.joint_idx==4,3:4)]);
    %left hand = 1,5,6,7
    lh_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==5,3:4);
                 frame_points(frame_points.joint_idx==6,3:4);
                 frame_points(frame_points.joint_idx==7,3:4)]);
    %right leg = 1,8,9,10
    rl_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==8,3:4);
                 frame_points(frame_points.joint_idx==9,3:4);
                 frame_points(frame_points.joint_idx==10,3:4)]);
    %left leg  = 1,11,12,13
    ll_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==11,3:4);
                 frame_points(frame_points.joint_idx==12,3:4);
                 frame_points(frame_points.joint_idx==13,3:4)]);
%     rh_points = table2array(rh_points);
%     lh_points = table2array(lh_points);
    
    subplot(2,2,4)
    plot(rh_points(:,1), rh_points(:,2), 'o-', 'LineWidth', 2,'color','r');
    set(gca, 'YDir','reverse')
    hold on
    plot(lh_points(:,1), lh_points(:,2), 'o-','LineWidth', 2,'color','b');
    plot(rl_points(:,1), rl_points(:,2), 'o-','LineWidth', 2,'color','r');
    plot(ll_points(:,1), ll_points(:,2), 'o-','LineWidth', 2,'color','b');
    hold off
    grid on 
    xlabel('Distance (mm)')
    ylabel('Distance (mm)')
    xlim([-250,200])
    ylim([-100,400])
    title("Body Joints Calibrated, Sim ref and smoothened")

body_points = pose_world;

total_frames = body_points.frame_num(end);
% figure();
    frame_points = body_points(body_points.frame_num ==f,:);
    
    %right hand = 1,2,3,4
    rh_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==2,3:4);
                 frame_points(frame_points.joint_idx==3,3:4);
                 frame_points(frame_points.joint_idx==4,3:4)]);
    %left hand = 1,5,6,7
    lh_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==5,3:4);
                 frame_points(frame_points.joint_idx==6,3:4);
                 frame_points(frame_points.joint_idx==7,3:4)]);
    %right leg = 1,8,9,10
    rl_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==8,3:4);
                 frame_points(frame_points.joint_idx==9,3:4);
                 frame_points(frame_points.joint_idx==10,3:4)]);
    %left leg  = 1,11,12,13
    ll_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==11,3:4);
                 frame_points(frame_points.joint_idx==12,3:4);
                 frame_points(frame_points.joint_idx==13,3:4)]);
%     rh_points = table2array(rh_points);
%     lh_points = table2array(lh_points);
    
    subplot(2,2,2)
    plot(rh_points(:,1), rh_points(:,2), 'o-', 'LineWidth', 2,'color','r');
    set(gca, 'YDir','reverse')
    hold on
    plot(lh_points(:,1), lh_points(:,2), 'o-','LineWidth', 2,'color','b');
    plot(rl_points(:,1), rl_points(:,2), 'o-','LineWidth', 2,'color','r');
    plot(ll_points(:,1), ll_points(:,2), 'o-','LineWidth', 2,'color','b');
    hold off
    grid on 
    xlabel('Distance (mm)')
    ylabel('Distance (mm)')
    xlim([-250,300])
    ylim([-100,400])
    title("Body Joints World dimensions")
    drawnow
    
    body_points = pose_sim;

    total_frames = body_points.frame_num(end);
% figure();
    frame_points = body_points(body_points.frame_num ==f,:);
    
    %right hand = 1,2,3,4
    rh_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==2,3:4);
                 frame_points(frame_points.joint_idx==3,3:4);
                 frame_points(frame_points.joint_idx==4,3:4)]);
    %left hand = 1,5,6,7
    lh_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==5,3:4);
                 frame_points(frame_points.joint_idx==6,3:4);
                 frame_points(frame_points.joint_idx==7,3:4)]);
    %right leg = 1,8,9,10
    rl_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==8,3:4);
                 frame_points(frame_points.joint_idx==9,3:4);
                 frame_points(frame_points.joint_idx==10,3:4)]);
    %left leg  = 1,11,12,13
    ll_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==11,3:4);
                 frame_points(frame_points.joint_idx==12,3:4);
                 frame_points(frame_points.joint_idx==13,3:4)]);
%     rh_points = table2array(rh_points);
%     lh_points = table2array(lh_points);
    
    subplot(2,2,3)
    plot(rh_points(:,1), rh_points(:,2), 'o-', 'LineWidth', 2,'color','r');
    set(gca, 'YDir','reverse')
    hold on
    plot(lh_points(:,1), lh_points(:,2), 'o-','LineWidth', 2,'color','b');
    plot(rl_points(:,1), rl_points(:,2), 'o-','LineWidth', 2,'color','r');
    plot(ll_points(:,1), ll_points(:,2), 'o-','LineWidth', 2,'color','b');
    hold off
    grid on 
    xlabel('Distance (mm)')
    ylabel('Distance (mm)')
    xlim([-250,200])
    ylim([-100,400])
    title("Body Joints Calibrated - Sim Reference")

end    


%%
body_points = pose_filt;

total_frames = body_points.frame_num(end);
figure();
for f = 0:total_frames
    frame_points = body_points(body_points.frame_num ==f,:);
    
    %right hand = 1,2,3,4
    rh_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==2,3:4);
                 frame_points(frame_points.joint_idx==3,3:4);
                 frame_points(frame_points.joint_idx==4,3:4)]);
    %left hand = 1,5,6,7
    lh_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==5,3:4);
                 frame_points(frame_points.joint_idx==6,3:4);
                 frame_points(frame_points.joint_idx==7,3:4)]);
    %right leg = 1,8,9,10
    rl_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==8,3:4);
                 frame_points(frame_points.joint_idx==9,3:4);
                 frame_points(frame_points.joint_idx==10,3:4)]);
    %left leg  = 1,11,12,13
    ll_points = table2array([frame_points(frame_points.joint_idx==1,3:4);
                 frame_points(frame_points.joint_idx==11,3:4);
                 frame_points(frame_points.joint_idx==12,3:4);
                 frame_points(frame_points.joint_idx==13,3:4)]);
%     rh_points = table2array(rh_points);
%     lh_points = table2array(lh_points);
    
    plot(rh_points(:,1), rh_points(:,2), 'o-', 'LineWidth', 2,'color','r');
    set(gca, 'YDir','reverse')
    hold on
    plot(lh_points(:,1), lh_points(:,2), 'o-','LineWidth', 2,'color','b');
    plot(rl_points(:,1), rl_points(:,2), 'o-','LineWidth', 2,'color','r');
    plot(ll_points(:,1), ll_points(:,2), 'o-','LineWidth', 2,'color','b');
    hold off
    grid on 
    xlabel('Distance (mm)')
    ylabel('Distance (mm)')
    xlim([-250,200])
    ylim([-100,400])
    title("Body Joints Calibrated, Sim ref and smoothened")
    drawnow
end 
%% plot time series
r_ankle_x = pose_sim.x(pose_sim.joint_idx ==10);
r_ankle_y = pose_sim.y(pose_sim.joint_idx ==10);

figure();
subplot(2,1,1)
plot(r_ankle_x);
title("Ankle X position")

subplot(2,1,2)
plot(r_ankle_y);
title("Ankle Y position")
suptitle("End-Effector position in Body Frame "+"Limb : Right Leg")

