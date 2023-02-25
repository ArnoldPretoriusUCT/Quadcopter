%% POSITION
p = x.data(:,1:3);
p_ = r_des.data;
t = x.time;

figure(1),clf,hold on,grid on

for i=1:length(p)
    if( p(i,3)<0 )
        p(i,3) = 0;
    end
end
%x
plot(t,p(:,1),'r',lineWidth=2)
plot(t,p_(:,1),'--r',lineWidth=2)
%y
plot(t,p(:,2),'b',lineWidth=2),
plot(t,p_(:,2),'--b',lineWidth=2)
%z
plot(t,p(:,3),'k',lineWidth=2)
plot(t,p_(:,3),'--k',lineWidth=2)

legend('$p_x$','$p^*_x$','$p_y$','$p_y^*$','$p_z$','$p_z^*$','Interpreter','latex',fontSize=20,orientation='horizontal')
xlabel('time (seconds)', fontSize=15)
ylabel('inertial-frame position (metres)', fontSize=15)
%% POSITION 3D
figure(2),clf
plot3( p(:,1), p(:,2), p(:,3) )

%% EULER ANGLES
q = x.data(:,7:10);
[angz, angy, angx] = quat2angle(q,'ZYX');
angx = angx*180/pi;
angy = angy*180/pi;
angz = angz*180/pi;

t = x.time;

figure(1),clf,hold on,grid on
%x
plot(t,angx,'r',lineWidth=2)
% plot(t,p_(:,1),'--r',lineWidth=2)
%y
plot(t,angy,'b',lineWidth=2),
% plot(t,p_(:,2),'--b',lineWidth=2)
%z
plot(t,angz,'k',lineWidth=2)
% plot(t,p_(:,3),'--k',lineWidth=2)

legend('$\phi$','$\theta$','$\psi$','Interpreter','latex',fontSize=20,orientation='horizontal')
xlabel('time (seconds)', fontSize=15)
ylabel('Euler angle (degrees)', fontSize=15)
%% THRUST
t = x.time;

figure(1),clf,hold on,grid on

ylabel('rotor thrust (Newtons)', fontSize=15)
%f1
subplot(4,1,1),hold on
plot(t,thrust_des.data(:,1),'-k',lineWidth=2)
plot(t,thrust.data(:,1),'b',lineWidth=2)

xlim([1 20])
ylabel('$f_1$ (N)', fontSize=15, Interpreter='latex')
%f2
subplot(4,1,2),hold on
plot(t,thrust_des.data(:,2),'-k',lineWidth=2)
plot(t,thrust.data(:,2),'b',lineWidth=2),

xlim([1 20])
ylabel('$f_2$ (N)', fontSize=15, Interpreter='latex')
%f3
subplot(4,1,3),hold on
plot(t,thrust_des.data(:,3),'-k',lineWidth=2)
plot(t,thrust.data(:,3),'b',lineWidth=2)

xlim([1 20])
ylabel('$f_3$ (N)', fontSize=15, Interpreter='latex')
%f4
subplot(4,1,4),hold on
plot(t,thrust_des.data(:,4),'-k',lineWidth=2)
plot(t,thrust.data(:,4),'b',lineWidth=2)

xlim([1 20])
ylabel('$f_4$ (N)', fontSize=15, Interpreter='latex')
xlabel('time (seconds)', fontSize=15)
% legend('$p_x$','$p^*_x$','$p_y$','$p_y^*$','$p_z$','$p_z^*$','Interpreter','latex',fontSize=20,orientation='horizontal')
%% BlADE ANGLE
t = x.time;

figure(1),clf,hold on,grid on

ylabel('rotor thrust (Newtons)', fontSize=15)
%f1
subplot(4,1,1),hold on
% plot(t,thrust_des.data(:,1),'-k',lineWidth=2)
plot(t,sigma.data(:,1)/6700*180/pi,'b',lineWidth=2)

xlim([1 20])
ylabel('$\sigma_1$', fontSize=15, Interpreter='latex')
%f2
subplot(4,1,2),hold on
% plot(t,thrust_des.data(:,2),'-k',lineWidth=2)
plot(t,sigma.data(:,2)/6700*180/pi,'b',lineWidth=2),

xlim([1 20])
ylabel('$\sigma_2$', fontSize=15, Interpreter='latex')
%f3
subplot(4,1,3),hold on
% plot(t,thrust_des.data(:,3),'-k',lineWidth=2)
plot(t,sigma.data(:,3)/6700*180/pi,'b',lineWidth=2)

xlim([1 20])
ylabel('$\sigma_3$', fontSize=15, Interpreter='latex')
%f4
subplot(4,1,4),hold on
% plot(t,thrust_des.data(:,4),'-k',lineWidth=2)
plot(t,sigma.data(:,4)/6700*180/pi,'b',lineWidth=2)

xlim([1 20])
ylabel('$\sigma_4$', fontSize=15, Interpreter='latex')
xlabel('time (seconds)', fontSize=15)
% legend('$p_x$','$p^*_x$','$p_y$','$p_y^*$','$p_z$','$p_z^*$','Interpreter','latex',fontSize=20,orientation='horizontal')

