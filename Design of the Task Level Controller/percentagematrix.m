function R = percentagematrix(per)


R = rand(3,3);
for i = 1:3
    for j = 1:3
        if R(i,j) >= 0.5
            R(i,j) = (1 + (per/100));
        else
            R(i,j) = (1 - (per/100));
        end
    end
end

end
