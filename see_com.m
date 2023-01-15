figure;
hold on;
for i = 1:4 
        plot(record_time_stamp,record_com(:,i))
end
legend('throtle','roll','pitch','yaw');