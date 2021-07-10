%filename = '2021-04-29 22-03-49.txt'
%filename = '2021-06-23 22-04-53_basketball.txt'
%filename = '2021-06-23 22-00-09_walk.txt'
%filename = '2021-06-23 22-07-19_mouse.txt'
%filename = '2021-06-23 22-19-19_eat.txt'
%filename = '2021-06-23 22-22-19_raquet.txt'
%filename = '2021-06-23 22-28-31_tap.txt'
%filename = '2021-06-23 22-30-16_walk.txt'
filename = '2021-06-23 22-34-40_run.txt'
data = load(filename);
[r,c] = size(data);
d = data(:,:);
type = d(:,1);
rot = d(:,2:10);
acc = d(:,11:14);
mag = d(:,15:17);
gyro = d(:,18:20);
heading = d(:,21:22);
apr = d(:,23:25);
gyro_apr = d(:,26:28);
tstamp = d(:,29);

mag_t1 = mag(1,:);
nl_1 = mag_t1.';
% nl_1 = mag_t1.'
% o_R = rotationMatrix(gyro_apr(r,1),gyro_apr(r,2),gyro_apr(r,3));
% nl_4 = o_R\mag_t0.'
% rotm_a = axisAngle2RotMat_a(nl_4, nl_1, 0.01);
% nl_new = rotm_a*nl_4

arrGyro = []; %inv(o_R)*mag_t0 = nl_4
arrGyro2 = []; %inv(o_R)*mag_t0 = nl_4
arrMag = []; %mag_t1.' = nl_1
arrMuse = []; %nl_new
arrMuse2 = []; %nl_new

lenGyro = 1;
lenGyro2 = 1;
lenMag = 1;
lenMuse = 1;
lenMuse2 = 1;

mag_t0 = mag(1,:);


for i = 1:r
    if(type(i) == 0)
        if(i == 1)
            aprg = gyro_apr(i,:);
        else
            aprg = [gyro_apr(i,1)-gyro_apr(i-1,1),gyro_apr(i,2)-gyro_apr(i-1,2),gyro_apr(i,3)-gyro_apr(i-1,3)];
        end
        %gyro
        o_R = rotationMatrix(aprg(1), aprg(2), aprg(3));
        if(lenGyro == 1)  nl_4 = o_R\mag_t0.';
        else  nl_4 = o_R\arrGyro(lenGyro-1,1:3).';    %이전 자이로값
        end
        ag = [nl_4(1), nl_4(2), nl_4(3), tstamp(i), degDiff(nl_1,nl_4)];
        arrGyro(lenGyro,:) = ag;
        lenGyro = lenGyro+1;
        
        %gyro
        o_R2 = rotationMatrix2(aprg(1), aprg(2), aprg(3));
        if(lenGyro2 == 1)  nl_4_2 = o_R2\mag_t0.';
        else  nl_4_2 = o_R2\arrGyro2(lenGyro2-1,1:3).';    %이전 자이로값
        end
        ag2 = [nl_4_2(1), nl_4_2(2), nl_4_2(3), tstamp(i), degDiff(nl_1,nl_4_2)];
        arrGyro2(lenGyro2,:) = ag2;
        lenGyro2 = lenGyro2+1;
        
        %muse
        o_R = rotationMatrix(aprg(1), aprg(2), aprg(3));
        if(lenMuse == 1)  nl_m = o_R\mag_t0.';
        else  nl_m = o_R\arrMuse(lenMuse-1,1:3).';	%gyro와 같음
        end
        %rotm_a = axisAngle2RotMat_a(nl_m, nl_1, 1);
        %nl_new = rotm_a*nl_m;	%보정하기
        amz = [nl_m(1), nl_m(2), nl_m(3), tstamp(i), degDiff(nl_1,nl_m)];
        arrMuse(lenMuse,:) = amz;
        lenMuse = lenMuse+1;
        
        %muse2
        o_R2 = rotationMatrix2(aprg(1), aprg(2), aprg(3));
        if(lenMuse2 == 1)  nl_m_2 = o_R2\mag_t0.';
        else  nl_m_2 = o_R2\arrMuse2(lenMuse2-1,1:3).';	%gyro와 같음
        end
        %rotm_a = axisAngle2RotMat_a(nl_m, nl_1, 1);
        %nl_new = rotm_a*nl_m;	%보정하기
        amz2 = [nl_m_2(1), nl_m_2(2), nl_m_2(3), tstamp(i), degDiff(nl_1,nl_m_2)];
        arrMuse2(lenMuse2,:) = amz2;
        lenMuse2 = lenMuse2+1;
        
    else 
        %magn
        mag_t1 = mag(i,:);
        nl_1 = mag_t1.';
        am = [nl_1(1), nl_1(2), nl_1(3), tstamp(i)];
        arrMag(lenMag,:) = am;
        lenMag = lenMag+1;
        
        %muse
        o_R = rotationMatrix(apr(i,1), apr(i,2), apr(i,3));
        if(lenMuse == 1)  nl_m = o_R\mag_t0.';
        else  nl_m = o_R\arrMuse(lenMuse-1,1:3).';	%gyro와 같음
        end
        rotm_a = axisAngle2RotMat_a(nl_m, nl_1, 1); 
        nl_new = rotm_a*nl_m;	%보정하기
        amz = [nl_new(1), nl_new(2), nl_new(3), tstamp(i), degDiff(nl_1,nl_new)];
        arrMuse(lenMuse,:) = amz;
        lenMuse = lenMuse+1;
        
        %muse2
        o_R2 = rotationMatrix2(apr(i,1), apr(i,2), apr(i,3));
        if(lenMuse2 == 1)  nl_m_2 = o_R2\mag_t0.';
        else  nl_m_2 = o_R2\arrMuse2(lenMuse2-1,1:3).';	%gyro와 같음
        end
        rotm_a = axisAngle2RotMat_a(nl_m_2, nl_1, 1); 
        nl_new_2 = rotm_a*nl_m_2;	%보정하기
        amz2 = [nl_new_2(1), nl_new_2(2), nl_new_2(3), tstamp(i), degDiff(nl_1,nl_new_2)];
        arrMuse2(lenMuse2,:) = amz2;
        lenMuse2 = lenMuse2+1;

    end
end


% figure('Name','time-x(1)','NumberTitle','off')
% plot(arrGyro(:,4),arrGyro(:,1),'-.go')
% hold on
% plot(arrMag(:,4),arrMag(:,1),'-.b*')
% hold on
% plot(arrMuse(:,4),arrMuse(:,1),':ms')
% legend('Gyro','Mag','Muse')
% 
% figure('Name','x-y(1)','NumberTitle','off')
% plot(arrGyro(:,2),arrGyro(:,1),'-.go')
% hold on
% plot(arrMag(:,2),arrMag(:,1),'-.b*')
% hold on
% plot(arrMuse(:,2),arrMuse(:,1),':ms')
% legend('Gyro','Mag','Muse')

figure('Name','time-error(1)','NumberTitle','off')
plot(arrGyro(:,4),arrGyro(:,5),'-.go')
hold on
plot(arrMuse(:,4),arrMuse(:,5),':ms')
legend('Gyro','Muse')

figure('Name','x-y-z(1)','NumberTitle','off')
plot3(arrGyro(:,3),arrGyro(:,2),arrGyro(:,1),'-.go')
hold on
plot3(arrMag(:,3),arrMag(:,2),arrMag(:,1),'-.b*')
hold on
plot3(arrMuse(:,3),arrMuse(:,2),arrMuse(:,1),':ms')
legend('Gyro','Mag','Muse')
% 
% % figure('Name','time-museErrType(1)','NumberTitle','off')
% % plot(tstamp(:),type(:)*5,'-.go')
% % hold on
% % plot(arrMuse(:,4),arrMuse(:,5),':ms')
% 
% 
% 
% figure('Name','time-x(2)','NumberTitle','off')
% plot(arrGyro2(:,4),arrGyro2(:,1),'-.co')
% hold on
% plot(arrMag(:,4),arrMag(:,1),'-.b*')
% hold on
% plot(arrMuse2(:,4),arrMuse2(:,1),':ms')
% legend('Gyro2','Mag','Muse2')
% 
% figure('Name','x-y(2)','NumberTitle','off')
% plot(arrGyro2(:,2),arrGyro2(:,1),'-.co')
% hold on
% plot(arrMag(:,2),arrMag(:,1),'-.b*')
% hold on
% plot(arrMuse2(:,2),arrMuse2(:,1),':ms')
% legend('Gyro2','Mag','Muse2')

% figure('Name','time-error(2)','NumberTitle','off')
% plot(arrGyro2(:,4),arrGyro2(:,5),'-.co')
% hold on
% plot(arrMuse2(:,4),arrMuse2(:,5),':ms')
% legend('Gyro2','Muse2')

% figure('Name','x-y-z(2)','NumberTitle','off')
% plot3(arrGyro2(:,3),arrGyro2(:,2),arrGyro2(:,1),'-.co')
% hold on
% plot3(arrMag(:,3),arrMag(:,2),arrMag(:,1),'-.b*')
% hold on
% plot3(arrMuse2(:,3),arrMuse2(:,2),arrMuse2(:,1),':ms')
% legend('Gyro2','Mag','Muse2')
% 
% 
% % figure('Name','time-museErrType(2)','NumberTitle','off')
% % plot(tstamp(:),type(:)*5,'-.co')
% % hold on
% % plot(arrMuse2(:,4),arrMuse2(:,5),':ms')
% 
% figure('Name','time-errorGyro(1&2)','NumberTitle','off')
% plot(arrGyro(:,4),arrGyro(:,5),'-.go')
% hold on
% plot(arrGyro2(:,4),arrGyro2(:,5),'-.co')
% legend('Gyro','Gyro2')
% 
% figure('Name','time-errorMuse(1&2)','NumberTitle','off')
% plot(arrMuse(:,4),arrMuse(:,5),':ms')
% hold on
% plot(arrMuse2(:,4),arrMuse2(:,5),':bs')
% legend('Muse','Muse2')