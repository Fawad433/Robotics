clc;
clear;
a = importdata('robot.key') %getting main matrix
Total_frames = a(1,2) % getting total frames from .key file 
key_frames = a(1,1)   % getting key frames from .key file 
K  = Original_Hermite(a,key_frames,Total_frames)
delete('robot.ang')
dlmwrite(fullfile('robot.ang'),Total_frames);
dlmwrite(fullfile('robot.ang'),K,'-append','delimiter','\t','precision',10);