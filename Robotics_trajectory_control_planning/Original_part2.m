clear;
clc;
a = importdata('object.key');
key_frames = a(1,1);
total_frames = a(1,2);
A1=a;
A1(:,4)=[];
A1(1,:) = [];

T = Original_Rotation(A1,1) %(.key matrix, no of rotaion matrix)
Q1 = R_to_Q(T);

T = Original_Rotation(A1,2) %(.key matrix, no of rotaion matrix)
Q2 = R_to_Q(T);

T = Original_Rotation(A1,3) %(.key matrix, no of rotaion matrix)
Q3 = R_to_Q(T);

T = Original_Rotation(A1,4) %(.key matrix, no of rotaion matrix)
Q4 = R_to_Q(T);

T = Original_Rotation(A1,5) %(.key matrix, no of rotaion matrix)
Q5 = R_to_Q(T);

QA = [Q1;Q2;Q3;Q4;Q5] % staking the quaternions

M = Q_interpolation(QA,key_frames,total_frames) % getting interpolated matrix
%M(1,:)=[]
L = 1;
for i= 1:total_frames
    
P1(L:L+2,1:3) = Q_to_R(M(i,:)); % Converting Quaternions back to rotation matrix
L = L +3;

end
key_Matrix =a;
key_Matrix(:,[1 2 3])=[];
key_Matrix(1,:) = [];
Final = Original_Catmull(key_Matrix,key_frames,total_frames); % doing catmull for cup position

PF = [P1 Final]; % stacking the rotaion and position matrix
PF = [PF(:,1),PF(:,2),PF(:,3),PF(:,4)];
delete('object.traj')
dlmwrite(fullfile('object.traj'),67);
dlmwrite('object.traj',PF,'-append','delimiter','\t','precision',10)
