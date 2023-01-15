figure;
hold on;
for i = 1:4 
        plot(record_time_stamp,record_Output_channel_data(:,i))
end
legend('Channel 1','Channel 2','Channel 3','Channel 4');