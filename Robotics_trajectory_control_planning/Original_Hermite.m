function OUT=Original_Hermite(key_Matrix,key_frames,total_frames)
K(1,1)=67; %introducing first row
for i=1: 1 : 6 % because there are 6 robot parts
    L=2; % it is 
    u=1; % it is one because i am using u=u-1 afterwards
    for j=2:1:size(key_Matrix,1) % started from 2nd because first row has no info
        
        if j==2
            Pi = key_Matrix(j,i) % only for the first time 
        end
        
        if j==3
            Di = key_Matrix(j,i) % only for the first time 
        end
        if mod(j,2) ~= 0 && j~=3    % taking odd values means derivative
            %if j==5 || j==7 || j==9 || j==11
            Df = key_Matrix(j,i) 
        end
        if mod(j,2) == 0 && j~=2 %taking even values means position
            %if j==4 || j==6 || j==8 || j==10
            Pf = key_Matrix(j,i)
           
        end
        if mod(j,2) ~= 0 && j~=3 
            %if j==5 || j==7 || j==9 || j==11
            u = u-1;
            while u <= 1
                u
                h1 = (2*u^3)-(3*u^2)+1; % hermite 
                h2 = -(2*u^3)+(3*u^2);  % hermite 
                h3 = (u^3)-(2*u^2)+u;   % hermite 
                h4 = (u^3)-(u^2);   % hermite 
               R = (Pi*h1)+(Pf*h2)+(Di*h3)+(Df*h4); % hermite 
               K(L,i) = R % output matrix
               L = L + 1; % increment for next row
               u = u + ((key_frames-1)/(total_frames-1)); % 4 is because 4 paths btw 5 points and 66 is because 67 keypoints and zeroth doesn't count
            end
            Pi = Pf; % initial becomes final
            Di = Df; % initial becomes final
            
        end
       
       end
end
K(1,:)=[];
OUT = K;
end
