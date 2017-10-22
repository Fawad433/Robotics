%clear;
%clc;
%a = importdata('object.key')

function OUT=Original_Catmull(key_Matrix,key_frames,total_frames)
key = key_frames;
key_frames = key_frames + 3; 
L=1;

for i=1:1:(size(key_Matrix,1)/3)
    Pos(i,:)= [key_Matrix(L,1),key_Matrix(L+1,1),key_Matrix(L+2,1)];
    L=L+3;
end

Pos = [Pos(1,:);Pos]; %adding dublicate first point to make velocity zero
Pos = [Pos;Pos((size(key_Matrix,1)/3)+1,:)] 
Pos = [Pos;Pos((size(key_Matrix,1)/3)+1,:)] 
%adding dublicate last point
T=0.5;
s= (1-T)./2; %(P(M,j) can take only one axis at a time)
M=1;
u=1;
for i = 1:1:key_frames-3 %(4 paths,2 dummies)
  u = u-1;
    while u < 1
        
        for  j=1:3
        Pos1(M,j) = [u^3, u^2, u, 1]*([-s     2-s   s-2        s;2.*s   s-3   3-(2.*s)   -s;-s     0     s          0;0      1     0          0]*[Pos(i,j);Pos(i+1,j);Pos(i+2,j);Pos(i+3,j)]);
        end

        u = u + ((key-1)/(total_frames-1))
                M=M+1;
    end
end
Pos1
L=1;
for i = 1:3:(total_frames)*3
    
    PFF(i,1)=Pos1(L,1);
    PFF(i+1,1)=Pos1(L,2);
    PFF(i+2,1)=Pos1(L,3);
    L=L+1;
end

    OUT=PFF;
end
