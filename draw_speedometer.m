function draw_speedometer(currentSpeed)

    scale = 10;  % 기존보다 더 큰 값으로 확대
    maxSpeed = 100;
    minAngle = 225;
    maxAngle = -45;

    cla;                % === 기존 내용 제거 ===
    axis equal;
    hold on;
    axis off;

    % 확대된 계기판에 맞게 범위 재조정
    xlim([-1.3 1.3] * scale)
    ylim([-1.3 1.3] * scale)

    theta = linspace(0, 2*pi, 360);
    % 바깥 테두리 원
    fill(scale * 1.1 * cos(theta), scale * 1.1 * sin(theta), ...
         [0.8 0.8 0.8], 'EdgeColor', 'k', 'LineWidth', 3);

    % 안쪽 흰 배경 원
    fill(scale * 0.95 * cos(theta), scale * 0.95 * sin(theta), ...
         'w', 'EdgeColor', 'none');

    % 눈금 및 숫자 표시
    for speed = 0:10:maxSpeed
        angle = deg2rad(minAngle - (minAngle - maxAngle) * (speed / maxSpeed));
        xOuter = scale * cos(angle);
        yOuter = scale * sin(angle);
        xInner = 0.85 * scale * cos(angle);
        yInner = 0.85 * scale * sin(angle);
        plot([xInner, xOuter], [yInner, yOuter], 'k', 'LineWidth', 2)

        % 숫자
        text(0.75 * scale * cos(angle), 0.75 * scale * sin(angle), num2str(speed), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 8, 'FontWeight', 'bold');
    end

    % 중심 원
    c = 0.05 * scale;
    rectangle('Position', [-c -c 2*c 2*c], 'Curvature', [1 1], 'FaceColor', 'k');

    % 속도 텍스트
    text(0, -1.5 * scale, [num2str(currentSpeed) ' m/s'], ...
        'HorizontalAlignment', 'center', 'FontSize', 18, 'FontWeight', 'bold');

    % 속도 바늘
    speedAngle = deg2rad(minAngle - (minAngle - maxAngle) * (currentSpeed / maxSpeed));
    xNeedle = [0, 0.7 * scale * cos(speedAngle)];
    yNeedle = [0, 0.7 * scale * sin(speedAngle)];
    plot(xNeedle, yNeedle, 'r', 'LineWidth', 2)
end
