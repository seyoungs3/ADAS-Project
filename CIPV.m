clc
close all
clear all
clear cipv_candidate_counter
load testset1.mat

x_temp = 0:0.1:60;

% 차선 정보
c0_l = LaneMarkPosition_Lh_ME;
c1_l = LaneMarkHeadAngle_Lh_ME;
c2_l = LaneMarkModelA_Lh_ME;
c3_l = LaneMarkModelDerivA_Lh_ME;
qual_l = LaneMarkQuality_Lh_ME;
type_l = LaneMarkType_Lh_ME;
vr_l = LaneMarkViewRange_Lh_ME;

c0_r = LaneMarkPosition_Rh_ME;
c1_r = LaneMarkHeadAngle_Rh_ME;
c2_r = LaneMarkModelA_Rh_ME;
c3_r = LaneMarkModelDerivA_Rh_ME;
qual_r = LaneMarkQuality_Rh_ME;
type_r = LaneMarkType_Rh_ME;
vr_r = LaneMarkViewRange_Rh_ME;

tunnel_idx = find(Tunnel==1);
PosLon_tunnel = PosLon(tunnel_idx);
PosLat_tunnel = PosLat(tunnel_idx);

scrsz = get(0, "ScreenSize");
figure("Position", [scrsz(3)/20 scrsz(4)*2/10 scrsz(3)*13/20 scrsz(4)*7/10]);

angles = linspace(0, 2*pi, 720);
radius = 1.5;
xCenter = 0;
yCenter = 0;
x = radius * cos(angles) + xCenter;
y = radius * sin(angles) + yCenter;



rel_dist_log = [];
rel_vel_log = [];
i_log = [];
ttc_level_log = [];
road_slope_deg = zeros(size(AnglePitch));

for i = 64000:100:240000
    ttc_stage = 0;  % ← 이 줄 추가!

    global cipv_candidate_counter
    if isempty(cipv_candidate_counter)
        cipv_candidate_counter = zeros(1, 10);  % 차량 수 최대 10대 가정
    end
    subplot(6,5,[1 2 3 4 5 6 7 8 9 10])
    x_l_vr = 0:0.1:vr_l(i);
    x_r_vr = 0:0.1:vr_r(i);

    y_l_temp = c0_l(i) + c1_l(i)*x_temp + c2_l(i)*x_temp.^2 + c3_l(i)*x_temp.^3;
    y_r_temp = c0_r(i) + c1_r(i)*x_temp + c2_r(i)*x_temp.^2 + c3_r(i)*x_temp.^3;

    y_l_vr = c0_l(i) + c1_l(i)*x_l_vr + c2_l(i)*x_l_vr.^2 + c3_l(i)*x_l_vr.^3;
    y_r_vr = c0_r(i) + c1_r(i)*x_r_vr + c2_r(i)*x_r_vr.^2 + c3_r(i)*x_r_vr.^3;

    lane_l_qual = num2str(qual_l(i));
    lane_r_qual = num2str(qual_r(i));

    % 주행 예측 경로
    c2_pred(i) = YAW_RATE(i) * pi/180 / Speed2D(i) / 2;
    y_path = c2_pred(i) * x_temp.^2;

    draw_veh(0, 0, -pi/2, 2, 4.5, 'b', 1);
    hold on
    grid on

    % 차량 위치 데이터
    veh_posX = [PosX01(i), PosX02(i), PosX03(i), PosX04(i), PosX05(i), PosX06(i), PosX07(i), PosX08(i)];
    veh_posY = [PosY01(i), PosY02(i), PosY03(i), PosY04(i), PosY05(i), PosY06(i), PosY07(i), PosY08(i)];

    cipv_idx = -1;
    min_dist = inf;

    cipv_idx = 0;
    min_dist = inf;

    for k = 1:length(veh_posX)
        x_obj = veh_posX(k);
        y_obj = veh_posY(k);

        if x_obj > 0 && x_obj <= max(x_temp)
            y_pred = interp1(x_temp, y_path, x_obj, 'linear', 'extrap');
            y_left = interp1(x_temp, y_l_temp, x_obj, 'linear', 'extrap');
            y_right = interp1(x_temp, y_r_temp, x_obj, 'linear', 'extrap');

            % 차선 좌우 반전 보정
            if y_left < y_right
                tmp = y_left;
                y_left = y_right;
                y_right = tmp;
            end

            % 조건 만족 여부
            if y_obj < y_left && y_obj > y_right && abs(y_obj - y_pred) < 3.0
                cipv_candidate_counter(k) = cipv_candidate_counter(k) + 1;
            else
                cipv_candidate_counter(k) = 0;
            end

            % 누적 조건 만족한 경우에만 CIPV 후보로 간주
            if cipv_candidate_counter(k) >= 5  % 약 2초 (10Hz 기준)
                dist = sqrt(x_obj^2 + y_obj^2);
                if dist < min_dist
                    min_dist = dist;
                    cipv_idx = k;
                end
            end
        end
    end
    % 차량 시각화
    for k = 1:length(veh_posX)
        if k == cipv_idx
            draw_veh(veh_posX(k), veh_posY(k), -pi/2, 2, 4.5, 'g', 2); % CIPV 강조
        else
            draw_veh(veh_posX(k), veh_posY(k), -pi/2, 2, 4.5, 'k', 1);
        end
    end

    % 차선 및 예측 경로 시각화
    plot(x_temp, -y_l_temp, 'k--')
    plot(x_temp, -y_r_temp, 'k--')
    plot(x_l_vr, -y_l_vr, 'g')
    plot(x_r_vr, -y_r_vr, 'g')
    plot(x_temp, y_path, 'c--')

    text(3, 3, lane_l_qual)
    text(3, -3, lane_r_qual)
    xlim([-60 120])
    ylim([-15 15])
    hold off
    drawnow



    % 나머지 subplot 시각화는 그대로 유지


    %%
    rel_vel = 0; ttc = inf; ttc_predicted = inf;
    warning_level = ''; focus_warning = ''; slip_warning = ''; warning_color = 'k';
    rel_dist = NaN;

    % ========== 운전자 집중도 평가 ==========
    steer_torque_threshold = 0.1;
    focus_time_window = 100;

    if i > focus_time_window
        recent_torque = CR_Mdps_DrvTq(i-focus_time_window:i);
        low_torque_ratio = sum(abs(recent_torque) < steer_torque_threshold) / focus_time_window;
        high_torque_ratio = sum(abs(recent_torque) > steer_torque_threshold) / focus_time_window;
    else
        low_torque_ratio = 0;
        high_torque_ratio = 0;
    end

    % GPS 기반 직선도로 판단 (벡터 간 방향 변화량 기준)
    if i > 10
        dx = PosLon(i) - PosLon(i-10);
        dy = PosLat(i) - PosLat(i-10);
        dx2 = PosLon(i-10) - PosLon(i-20);
        dy2 = PosLat(i-10) - PosLat(i-20);
        angle_diff = abs(atan2(dy, dx) - atan2(dy2, dx2));
    else
        angle_diff = 0;
    end

    is_straight = angle_diff < deg2rad(5);  % 직선도로 판단 기준
    is_curve = angle_diff > deg2rad(20);

    % 집중도 경고 조건
    if is_straight && high_torque_ratio > 0.9
        focus_warning = '주의! 운전자 집중도 낮음';
        warning_color = 'r';

    elseif is_curve && low_torque_ratio > 0.9
        focus_warning = '주의! 운전자 집중도 낮음';
        warning_color = 'r';
    else
        focus_warning = '';
    end


    if cipv_idx > 0
        % CIPV 위치
        x_cipv = veh_posX(cipv_idx);
        y_cipv = veh_posY(cipv_idx);
        rel_dist = sqrt(x_cipv^2 + y_cipv^2); % 유클리드 거리

        % CIPV 속도 (VelX01 ~ VelX08 변수 사용)
        cipv_velx_array = [VelX01(i), VelX02(i), VelX03(i), VelX04(i), VelX05(i), VelX06(i), VelX07(i), VelX08(i)];
        cipv_velx = cipv_velx_array(cipv_idx);
        % 내 차량 속도

        ego_velx = Speed2D(i);
        rel_vel = ego_velx - cipv_velx; % 상대속도 (정면 기준)

        % TTC 계산 (상대속도가sub0보다 클 때만)
        if rel_vel > 0
            ttc = rel_dist / rel_vel;
        else
            ttc = inf;
        end

        % 센서 예측 TTC
        ttc_c_array = [TTC_C_1(i), TTC_C_2(i), TTC_C_3(i), TTC_C_4(i), TTC_C_5(i), TTC_C_6(i), TTC_C_7(i), TTC_C_8(i)];
        ttc_predicted = ttc_c_array(cipv_idx);

        % TTC 기반 경고 판단
        if ttc < 1.5 || ttc_predicted < 1.5
            warning_level = '🚨 경고! (3단계)';
            warning_color = 'r';
            ttc_stage = 3;
        elseif ttc < 3.0 || ttc_predicted < 3.0
            warning_level = '⚠️ 주의 (2단계)';
            warning_color = 'm';
            ttc_stage = 2;
        elseif ttc < 5.0 || ttc_predicted < 5.0
            warning_level = '   경계 (1단계)';
            warning_color = [1.0,0.6,0.0];
            ttc_stage = 1;
        else
            warning_level = '정상';
            warning_color = 'g';
            ttc_stage = 0;
        end

        subplot(6,5,[1 2 3 4 5 6 7 8 9 10])


        text(-55, 18, ...
            sprintf('CIPV: #%d | TTC: %.1fs (Pred: %.1fs) | Δv: %.1f m/s | %s %s', ...
            cipv_idx, ttc, ttc_predicted, rel_vel, warning_level, focus_warning), ...
            'Color', warning_color, ...
            'FontSize', 13, ...
            'HorizontalAlignment', 'left', ...
            'FontWeight', 'bold');

        drawnow
    end
        %%

        subplot(6, 5, [11 16])
        % 현재 위치 인덱스 i는 정의되어 있다고 가정
        currentLat = PosLat(i);
        currentLon = PosLon(i);

        % 지도 위에 현재 위치만 표시
        geoplot(currentLat, currentLon, 'ro', 'LineWidth', 2, 'MarkerSize', 10);
        hold on
        geobasemap streets % 지도 스타일 설정

        % 줌 인: 현재 위치를 중심으로 좁은 범위 설정
        zoomRange = 0.001; % 숫자가 작을수록 더 확대됨
        geolimits([currentLat - zoomRange, currentLat + zoomRange], ...
            [currentLon - zoomRange, currentLon + zoomRange]);
        hold off;
        drawnow


        %%
        % hold on
        % 상대거리/속도 기록
        i_log(end+1) = i;
        rel_dist_log(end+1) = rel_dist;
        rel_vel_log(end+1) = rel_vel;
        % ttc_level_log(end+1) = ttc_stage;

        if cipv_idx > 0
            ttc_level_log(end+1) = ttc_stage;
        else
            ttc_level_log(end+1) = NaN;  % 또는 0 등으로 대체 가능
        end
        %%

        % 거리 그래프 subplot(2행 4열)
        subplot(6,5,[14 15 19 20] ) ;
        plot(i_log, rel_dist_log, 'b');
        title('CIPV 거리');
        xlabel('샘플');
        ylabel('거리 [m]');
        grid on;
        drawnow;
        %%
        % 속도 그래프 subplot(3행 4열)
        subplot(6,5,[24 25 29 30]) ;
        plot(i_log, rel_vel_log, 'r');
        title('상대 속도');
        xlabel('샘플');
        ylabel('Δv [m/s]');
        grid on;

        drawnow;

        % 도로 기울기 계산, 플롯
        road_slope_deg(i) = AnglePitch(i) * pi / 180;
        subplot(6,5,[22,23,27,28]);
        plot(road_slope_deg, 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Road Slope (°)');
        title('Road Inclination Analysis');
        grid on;
        ylim([-1 1]);
        drawnow;

        %%
        % subplot(5,5,7) - 배터리형 램프 3단계
        subplot(6,5,12);
        cla;
        axis off;
        hold on;

        % 박스 위치 설정
        box_x = 	[0.2, 0.4, 0.6];  % x 위치 (왼 → 오)
        box_y = 0.4;               % y 고정
        w = 0.2; h = 0.3;         % width, height

        % 색상 지정
        colors = {'[0.6 0.6 0.6]', '[0.6 0.6 0.6]', '[0.6 0.6 0.6]'}; % 기본 회색
        ttc_text = '미 인식';
        if ttc_stage == 1
            colors{1} = 'g';  % 초록 1칸
        elseif ttc_stage == 2
            colors{1} = 'y'; colors{2} = 'y';  % 노랑 2칸
        elseif ttc_stage == 3
            colors{1} = 'r'; colors{2} = 'r'; colors{3} = 'r';  % 빨강 3칸
        end

        % 박스 그리기
        for n = 1:3
            rectangle('Position', [box_x(n), box_y, w, h], ...
                'FaceColor', colors{n}, 'EdgeColor', 'k', 'LineWidth', 1.5);
        end
        title('TTC 단계 표시');

        %% 숫자 표시 개선
        %% 숫자 표시 개선
        subplot(6,5,13);
        cla;
        axis off;

        ttc_label = {'1 \color{green}안전', '2 \color[rgb]{1.0,0.6,0.0}주의', '3 \color{red}위험'};

        % ttc_label = {'\color{green}1 \color{green}✅정상', '\color[rgb]{1.0,0.6,0.0}2 \color[rgb]{1.0,0.6,0.0}⚠️주의', '\color{red}3 \color{red}🚨위험'};

        if ttc_stage >= 1 && ttc_stage <= 3
            ttc_text = ttc_label{ttc_stage};
        else
            ttc_text = '미 인식';
            colors = {'[0.6 0.6 0.6]', '[0.6 0.6 0.6]', '[0.6 0.6 0.6]'}; % 기본 회색
        end

        text(0.5, 0.5, ttc_text, 'FontSize', 20, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'Interpreter', 'tex');
        title('TTC 단계 숫자');

        hold off;
        drawnow;

    end