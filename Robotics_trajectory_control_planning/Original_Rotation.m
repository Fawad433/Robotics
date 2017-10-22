function OUT=Original_Rotation(rotation_matrix,No)
for i=1:3
    k = ((No-1)*3)+1;
        OUT(1,:)=rotation_matrix(k,:);
        OUT(2,:)=rotation_matrix(k+1,:);
        OUT(3,:)=rotation_matrix(k+2,:);
end
end