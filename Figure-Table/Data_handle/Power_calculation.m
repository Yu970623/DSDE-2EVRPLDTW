% 定义常数和参数
rho = 1.225; % 空气密度 kg/m^3
n = 4; % 扇叶个数
g = 9.81; % 重力加速度 m/s^2
W = 5; % 无人机自重 kg

% 扇叶半径范围
r1 = 10; % 扇叶半径范围，单位：厘米
zeta = pi .* r1.^2; % 对应的扇叶面积

x = 0:0.1:5;
% 计算公式结果
result1 = (W+x).^ (3/2) * g^3 ./ sqrt(2 * rho * zeta * n); % 第一种公式结果
result2 = (W+5)^ (3/2) * g^3 ./ sqrt(2 * rho * zeta * n); % 第二种公式结果

% (5+y).^ (3/2) * 9.81^3 / sqrt(2 * 1.225 * (pi .* 10.^2) * 4);


dis1 = 500./result1.*10;
dis2 = 500./result2.*10;
% 绘制曲线图
figure;

plot(x, dis1, 'r--', 'LineWidth', 2); % 第一种曲线，蓝色
% 
% % 添加标签和标题
% xlabel('运载货物重量 (Kg)');
% ylabel('实时功率');
% title('扇叶面积10cm^2下不同货物运载量下飞行功率');

% figure;
% plot(r1, result1, 'b', 'LineWidth', 2); % 第一种曲线，蓝色
% hold on
% plot(r1, result2, 'r--', 'LineWidth', 2); % 第二种曲线，红色虚线

% line([10,10],[0,1200],'lineStyle',':','LineWidth', 2)
% hold off;

% 添加标签和标题
xlabel('Weight of parcels transported (Kg)');
ylabel('Maximum range at full charge (Km)');
title('Four fans with blade area of (\pi \cdot 10^2)cm^2');
legend('Fan blade radius 10 cm');

% 添加标签和标题
% xlabel('Fan blade radius (cm)');
% ylabel('Power result (W/h)');
% title('Power variation under different blade areas');
% legend('Empty weight 5kg', 'Full load parcel 5kg');
% grid on;


% 添加标签和标题
% xlabel('Fan blade radius (cm)');
% ylabel('Maximum sailing distance');
% title('Variation of farthest flight distance under different blade areas');
% legend('Empty weight 5kg', 'Full load parcel 5kg');
grid on;
