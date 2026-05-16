close all
clear
clc

phd = load("phd_slam_timing.mat");
phd = phd.avg_compute_time;
cphd = load("cphd_slam_timing.mat");
cphd = cphd.avg_compute_time;

figure()
scatter(1:824,phd,"DisplayName","PHD-SLAM")
hold on
scatter(1:824,cphd,"DisplayName","CPHD-SLAM")
xlabel("Index")
ylabel("Compute time (s)")
modify_figure(20)
xlim([0, 824])
legend