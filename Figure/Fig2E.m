% 清除环境
clc;
clear;
close all;

% 左边3个点竖排位置
x_left = zeros(1, 3);
y_left = linspace(-4, 4, 3);

% 右边15个点以更开阔的弧形排列
theta = linspace(-pi/2, pi/2, 15); % 使用更大的跨度以形成更宽的弧形
radius = 4; % 弧形的半径
x_right = cos(theta) * radius + 2; % 右边位置，调整以与左边分开
y_right = sin(theta) * radius;

% 绘图
figure;
hold on;
axis equal;

% 绘制左边的3个点（六角形标记）
plot(x_left, y_left, 'h', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

% 绘制右边的15个点（灰色圆形标记）
plot(x_right, y_right, 'o', 'MarkerSize', 8, 'Color', [0.5, 0.5, 0.5]);

% 连线左边3个点与右边15个点
for i = 1:length(x_left)
    for j = 1:length(x_right)
        plot([x_left(i), x_right(j)], [y_left(i), y_right(j)], 'k-');
    end
end

% 连线右边15个点之间
for i = 1:length(x_right)
    for j = i+1:length(x_right)
        plot([x_right(i), x_right(j)], [y_right(i), y_right(j)], 'k--');
    end
end

% 设置坐标轴限制
xlim([-1, 10]);
ylim([-5, 5]);

hold off;