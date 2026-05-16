close all
clear
clc

load('mapping_cphd_ospa.mat');
cphd_ospa = ospa;

load('mapping_phd_ospa.mat');
phd_ospa = ospa;

figure()
plot(phd_ospa(1,:),phd_ospa(2,:),'DisplayName','PHD-SLAM', LineWidth=2)
hold on
plot(cphd_ospa(1,:),cphd_ospa(2,:),'DisplayName','CPHD-SLAM',LineWidth=2)
legend
xlabel("Time vec(s)")
ylabel("OSPA (m)")
modify_figure(20)