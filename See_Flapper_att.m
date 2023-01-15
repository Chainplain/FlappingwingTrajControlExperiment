figure;
hold on;
for i = 1:3 
    for j =1:3
        plot(record_time_stamp,record_Flapper_att(:,i,j))
    end
end