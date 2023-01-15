figure;

for i = 1:3 
        subplot(3,1,i)
        plot(record_time_stamp,record_p(:,i))
        hold on;
        plot(record_time_stamp,record_Observer_p(:,i))
end

figure;
for i =1:3
    subplot(3,1,i)
        plot(record_time_stamp,record_Observer_v(:,i));
        title('velocity');
end


figure;
for i =1:3
    subplot(3,1,i)
        plot(record_time_stamp,record_Observer_z(:,i));
        title('z');
end

figure;
for i =1:3
    subplot(3,1,i)
        plot(record_time_stamp,record_u_t(:,i));
        title('u');
end
