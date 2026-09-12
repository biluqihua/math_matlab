function problem3_dtsp_main()
% =========================================================================
% 问题3 全向干扰源动态搜索定位清除 (DTSP) —— 竞赛版 (对接模拟器 HTTP 接口)
% 通信层: 严格按附件《模拟器通信接口说明》 (4 条指令 /enter /measure /clear /exit)
% 算法层: 严格按提示词四阶段架构 (不加多余步骤)
%   Phase1 起点 S1(0,0) 全频道扫描 -> 落点信号 + 无响应扇形 Theta_blank
%   Phase2 自适应双点覆罩 (phi0,d0) -> S2 二次交会 -> 凸包/质心/R_k
%   Phase3 动态 DTSP + 实时 R_k 更新 + 三级清除响应
%   Phase4 终局切线收敛 + 极速早停 (停用 measure, 达 N_min 即 /exit)
%
% 附件关键物理规则 (本程序遵守):
%   频道 1..20; 干扰源总数 10~16; 接收半径 1000~1500m(未知);
%   near 阈值 5m; 清除半径 20m; 移动 5m/s; 检测动作 5s; 切频道 1s; 清除 3s/5s;
%   measure 返回 no_signal / near / direction; clear 返回 success / no_target_in_range;
%   虚拟限时 360000s, 现实限时取 /enter 返回的 remaining_real_duration_s。
% =========================================================================

    % ======================= 全局配置 =======================
    cf = struct();
    cf.rb   = '202604002043';   % robot_id 必须与当前登录参赛队号逐字节一致
    cf.url  = 'http://127.0.0.1:2026';
    cf.to   = 5;                      % HTTP 超时 s
    cf.N_ch = 20;                     % 频道 1..20
    cf.N_min= 10;                     % 清除下限 N_min
    cf.R_arena = 1800;                % 目标区域半径
    cf.R_near  = 5;                   % 近场阈值 (near)
    cf.R_conv  = 20;                  % 收敛阈值 R_k<=20m
    cf.R_clear = 20;                  % 清除半径
    cf.R_sense = 1300;                % 名义接收半径(真实1000~1500未知), 用于盲区几何
    cf.df_unc  = 1;                   % 示向度误差 +/-1 度 (楔形半角, 见题目正文)
    cf.d0_min  = 154.1;               % 步长下界
    cf.d_first = 400;                 % 单测向线目标首次沿射线逼近距离
    cf.min_blank_width = 10;          % 判定无响应扇形的最小角宽 deg
    cf.safety_real = 60;              % 现实时间安全余量 s
    cf.max_virtual = 360000;          % 虚拟世界限时 s
    cf.max_iter    = 2000;
    cf.animate = true;                % 实时可视化 (正式测试可设 false)

    op = weboptions('MediaType','application/json','Timeout',cf.to);
    fid = fopen('robot_dtsp_log.txt','w','n','UTF-8'); if fid < 0, fid = 1; end

    % ======================= 状态 =======================
    st = struct('x',0,'y',0,'vt',0,'rid',0,'nc',0,'rem',1200,'kn',0);
    st.path = [0 0];  st.cleared = zeros(0,2);  st.scans = zeros(0,2);
    st.last_bearings = [];  st.virt = [];

    % ======================= /enter =======================
    st.rid = st.rid + 1;
    q = struct('arena_id','default','robot_id',cf.rb,'request_id',sprintf('r%d',st.rid));
    r = post('/enter', q);
    if ~isfield(r,'accepted') || ~r.accepted
        fprintf('进入失败: %s\n', jsonencode(r)); if fid>1, fclose(fid); end; return;
    end
    st.rem = r.remaining_real_duration_s;  st.t0 = tic;  st.vt = r.virtual_time_s;
    fprintf('enter ok, 现实可用=%.0fs\n', st.rem);

    % ======================= TargetStruct =======================
    tp = struct('id',0, 'convex_hull',zeros(0,2), 'centroid',[NaN NaN], ...
                'radius_Rk',inf, 'is_cleared',false, 'detected',false, 'bearings',zeros(0,3));
    T = repmat(tp, cf.N_ch, 1);

    % ======================= 绘图引擎 =======================
    if cf.animate
        hfig = figure('Name','DTSP 清扫','Color','w','Position',[80 80 820 720]);
    else
        hfig = [];
    end

    % =====================================================================
    % 阶段一: 起点 S1(0,0) 第一次全频道扫描
    % =====================================================================
    fprintf('=== Phase1: S1 全频道扫描 ===\n');
    st.last_bearings = sweep(0, 0);
    update_all();
    fprintf('  N_known=%d  Cleared=%d\n', st.kn, st.nc);
    plot_engine(hfig);

    % =====================================================================
    % 阶段二: 自适应 (phi0,d0) -> S2 二次交会
    % =====================================================================
    fprintf('=== Phase2: 自适应 (phi0,d0) ===\n');
    [phi0, d0] = solve_adaptive_step([0 0], st.last_bearings);
    S2 = [d0*cosd(phi0), d0*sind(phi0)];
    fprintf('  phi0=%.1f deg, d0=%.1f m, S2=(%.0f,%.0f)\n', phi0, d0, S2(1), S2(2));
    st.last_bearings = sweep(S2(1), S2(2));
    update_all();
    fprintf('  N_known=%d  Cleared=%d\n', st.kn, st.nc);
    plot_engine(hfig);

    % =====================================================================
    % 阶段三: 动态 DTSP + 实时 R_k + 三级清除响应
    % =====================================================================
    fprintf('=== Phase3: 动态 DTSP + 三级清除 ===\n');
    iter = 0;
    while bud() && st.nc < cf.N_min && iter < cf.max_iter
        iter = iter + 1;
        update_all();                 % 每次驻留后实时更新所有未清除区域 R_k

        % 分支 B: 已知目标 < N_min -> 连片未知区域质心 V_explore 加入 DTSP 列表
        st.virt = [];
        if st.kn < cf.N_min
            st.virt = generate_virtual_explore_node();
        end

        % 贪心 DTSP: 融合已知质心与虚拟节点, 选最近
        [k, isv, dest] = plan_greedy_dtsp(st.virt);

        if isv
            % 前往虚拟探索节点盲探 (顺路拦截已收敛目标)
            intercept_clears(dest);
            st.last_bearings = sweep(dest(1), dest(2));
            update_all();
        elseif ~isempty(k)
            % Level 1: R_k>20m -> 前往质心停稳检测
            intercept_clears(T(k).centroid);
            [res, svd] = act_measure(T(k).centroid(1), T(k).centroid(2), k);
            if ~strcmp(res, 'direction')
                % Level 2: 质心无信号(no_signal)或 5m 近场(near) -> 原地 /clear
                act_clear(st.x, st.y, k);
            else
                % Level 1: 新测向线切削凸包, 大幅缩减区域面积
                T(k).bearings(end+1,:) = [st.x st.y svd];
                update_convex_hull(k);
            end
        else
            % 无待细化目标 (所有已知目标已收敛) -> 转入终局
            break;
        end

        if check_early_exit(), break; end
        plot_engine(hfig);
    end
    fprintf('  Phase3 结束: N_known=%d  Cleared=%d\n', st.kn, st.nc);

    % =====================================================================
    % 阶段四: 终局切线收敛 + 极速早停 (停用 /measure, 切线顺路清除)
    % =====================================================================
    fprintf('=== Phase4: 终局切线收敛 (停用 /measure) ===\n');
    terminal_tangent_clear(hfig);
    fprintf('  Phase4 结束: Cleared=%d\n', st.nc);

    % ======================= /exit =======================
    st.rid = st.rid + 1;
    q = struct('arena_id','default','robot_id',cf.rb,'request_id',sprintf('r%d',st.rid));
    r = post('/exit', q);
    if isfield(r,'accepted') && r.accepted && isfield(r,'exit_reason')
        fprintf('exit: %s\n', r.exit_reason);
        if isfield(r,'virtual_time_s'), st.vt = r.virtual_time_s; end
    end
    fprintf('===== 虚拟耗时=%.1fs  清除=%d =====\n', st.vt, st.nc);
    if st.nc >= cf.N_min
        fprintf('===== 达标: 已清除 %d >= N_min(%d) =====\n', st.nc, cf.N_min);
    else
        fprintf('===== 未达标: 仅清除 %d < N_min(%d) =====\n', st.nc, cf.N_min);
    end
    if fid > 1, fclose(fid); end

% =========================================================================
% ========================= 通信层 (附件) ================================
% =========================================================================

    % -------- HTTP POST + 幂等重试 (复用原 request_id) --------
    function r = post(path, q)
        for a = 1:3
            try
                r = webwrite([cf.url path], q, op);
                if ischar(r) || isstring(r), r = jsondecode(char(r)); end
                if fid > 1, fprintf(fid, '%s %s %s\n', path, jsonencode(q), jsonencode(r)); end
                return;
            catch e
                if a == 3 || contains(e.identifier, 'HTTP', 'IgnoreCase', true)
                    r = struct('accepted', false, 'err', e.message); return;
                end
                pause(1);   % 网络抖动 -> 复用同一 q/request_id 重试
            end
        end
    end

    % -------- 基础请求体 (新动作新 request_id) --------
    function q = base_req()
        st.rid = st.rid + 1;
        q = struct('arena_id','default','robot_id',cf.rb,'request_id',sprintf('r%d',st.rid));
    end

    % -------- /measure: 到(x,y)对频道 ch 检测 --------
    function [res, svd] = act_measure(x, y, ch)
        q = base_req();
        q.position = struct('x', x, 'y', y);
        q.channel  = ch;
        r = post('/measure', q);
        if ~isfield(r,'accepted') || ~r.accepted
            res = 'no_signal'; svd = NaN; return;   % 未执行 -> 视为无信号
        end
        st.x = x;  st.y = y;  st.vt = r.virtual_time_s;
        st.path(end+1,:) = [x y];
        if isfield(r, 'svd_deg'), svd = r.svd_deg; else, svd = NaN; end
        res = r.measure_result;
    end

    % -------- /clear: 到(x,y)清除频道 ch 的干扰源 --------
    function ok = act_clear(x, y, ch)
        q = base_req();
        q.position = struct('x', x, 'y', y);
        q.channel  = ch;
        r = post('/clear', q);
        if ~isfield(r,'accepted') || ~r.accepted
            ok = false; return;
        end
        st.x = x;  st.y = y;  st.vt = r.virtual_time_s;
        st.path(end+1,:) = [x y];
        ok = isfield(r,'clear_result') && strcmp(r.clear_result, 'success');
        if ok
            T(ch).is_cleared = true;
            st.nc = st.nc + 1;
            st.cleared(end+1,:) = [x y];        % 清除位置 (红叉)
            fprintf('  [clear] ch%d @(%.0f,%.0f)  Cleared=%d\n', ch, x, y, st.nc);
        end
    end

    % -------- 全频道扫描: 在(x,y)依次 /measure 频道1..20 --------
    function bearings = sweep(x, y)
        bearings = [];
        for ch = 1:cf.N_ch
            if ~bud(), break; end
            if T(ch).is_cleared, continue; end
            [res, svd] = act_measure(x, y, ch);
            if strcmp(res, 'direction')
                T(ch).id = ch;  T(ch).detected = true;
                T(ch).bearings(end+1,:) = [x y svd];
                bearings(end+1) = svd; %#ok<AGROW>
            elseif strcmp(res, 'near')
                T(ch).id = ch;  T(ch).detected = true;
                act_clear(x, y, ch);            % 5m 近场 -> 原地清除
            end
            % no_signal -> 无响应扇形
        end
        st.scans(end+1,:) = [x y];              % 记录扫描位置
        st.kn = sum([T.detected]);
    end

    % -------- 现实+虚拟时间预算 --------
    function y = bud()
        y = (st.rem - toc(st.t0)) > cf.safety_real && st.vt < cf.max_virtual;
    end

    % -------- Level 3 顺路拦截: 沿去 dest 的路径清除收敛目标 --------
    function intercept_clears(dest)
        A = [st.x st.y];  list = [];
        for ch = 1:cf.N_ch
            if T(ch).detected && ~T(ch).is_cleared && isfinite(T(ch).centroid(1)) ...
               && T(ch).radius_Rk <= cf.R_conv
                [Pc, d] = get_projection_point(A, dest, T(ch).centroid);
                if d <= cf.R_clear          % 路径经过其 20m 边界 -> 顺路 /clear
                    list(end+1,:) = [Pc norm(Pc-A) ch]; %#ok<AGROW>
                end
            end
        end
        if ~isempty(list)
            list = sortrows(list, 3);
            for i = 1:size(list,1)
                p = list(i,1:2);  ch = list(i,4);
                act_clear(p(1), p(2), ch);    % 绝不去质心, 顺路拦截
            end
        end
    end

% =========================================================================
% ========================= 算法层 (提示词) ==============================
% =========================================================================

    % -------- 已知目标计数 N_known --------
    function n = count_known()
        n = sum([T.detected]);
    end

    % -------- 实时更新所有未清除目标凸包/质心/R_k --------
    function update_all()
        for ch = 1:cf.N_ch
            if T(ch).detected && ~T(ch).is_cleared
                update_convex_hull(ch);
            end
        end
    end

    % -------- 核心函数: update_convex_hull(切削凸包+质心+R_k) --------
    function update_convex_hull(ch)
        nb = size(T(ch).bearings, 1);
        if nb == 0
            T(ch).convex_hull = zeros(0,2);  T(ch).centroid = [NaN NaN];  T(ch).radius_Rk = inf;
        elseif nb == 1
            T(ch).convex_hull = intersect_bearings(T(ch).bearings);
            x = T(ch).bearings(1,1); y = T(ch).bearings(1,2); th = T(ch).bearings(1,3);
            d = min(cf.d_first, cf.R_sense);
            T(ch).centroid = [x + d*cosd(th), y + d*sind(th)];
            T(ch).radius_Rk = inf;
        else
            T(ch).convex_hull = intersect_bearings(T(ch).bearings);
            [T(ch).centroid, T(ch).radius_Rk] = compute_centroid_radius(T(ch).convex_hull);
        end
    end

    % -------- 核心函数: intersect_bearings(半平面交凸包重构) --------
    function P = intersect_bearings(b)
        P = circle_polygon(0, 0, cf.R_arena, 64);
        n = size(b,1);
        for i = 1:n
            th = b(i,3);  Sx = b(i,1);  Sy = b(i,2);
            a1 = th - cf.df_unc;  a2 = th + cf.df_unc;
            P = clip_halfplane(P, sind(a2), -cosd(a2), -sind(a2)*Sx + cosd(a2)*Sy);
            P = clip_halfplane(P, -sind(a1), cosd(a1), sind(a1)*Sx - cosd(a1)*Sy);
            if isempty(P), break; end
        end
    end

    function P = clip_halfplane(P, A, B, C)
        if isempty(P), return; end
        Q = zeros(0,2);  m = size(P,1);
        for i = 1:m
            c = P(i,:);  nx = P(mod(i,m)+1,:);
            fc = A*c(1) + B*c(2) + C;  fn = A*nx(1) + B*nx(2) + C;
            if fc >= 0 && fn >= 0
                Q(end+1,:) = nx;                                      %#ok<AGROW>
            elseif fc >= 0 && fn < 0
                t = fc/(fc-fn);  Q(end+1,:) = c + t*(nx-c);           %#ok<AGROW>
            elseif fc < 0 && fn >= 0
                t = fc/(fc-fn);  Q(end+1,:) = c + t*(nx-c);  Q(end+1,:) = nx; %#ok<AGROW>
            end
        end
        P = Q;
    end

    function P = circle_polygon(cx, cy, R, N)
        th = linspace(0, 360, N+1)';  th(end) = [];
        P = [cx + R*cosd(th), cy + R*sind(th)];
    end

    % -------- 面积质心(鞋带公式) + 外接圆半径 R_k --------
    function [g, Rk] = compute_centroid_radius(P)
        m = size(P,1);
        if m == 0, g = [NaN NaN];  Rk = inf;  return; end
        if m < 3, g = mean(P,1);  Rk = max(sqrt(sum((P-g).^2,2)));  return; end
        A = 0;  Cx = 0;  Cy = 0;
        for i = 1:m
            j = mod(i,m)+1;
            cr = P(i,1)*P(j,2) - P(j,1)*P(i,2);
            A = A + cr;  Cx = Cx + (P(i,1)+P(j,1))*cr;  Cy = Cy + (P(i,2)+P(j,2))*cr;
        end
        A = A/2;
        if abs(A) < 1e-9, g = mean(P,1);  Rk = max(sqrt(sum((P-g).^2,2)));  return; end
        g = [Cx/(6*A), Cy/(6*A)];
        Rk = max(sqrt(sum((P - g).^2, 2)));
    end

    % -------- 核心函数: solve_adaptive_step(phi0 角平分线 + d0 盲区最小) --------
    function [phi0, d0] = solve_adaptive_step(S1, Measure1)
        bs = compute_blank_sectors(Measure1);
        if isempty(bs)
            phi0 = 45;
        else
            phi0 = bs(1).bisector;          % 最大无响应扇形角平分线
        end
        d0 = optimize_d0(S1, phi0);         % 自适应步长: 最小化盲区面积
    end

    function d0 = optimize_d0(S1, phi0)
        n = 60;  d0s = linspace(cf.d0_min, cf.R_sense, n);
        bestA = inf;  d0 = cf.d0_min;
        for i = 1:n
            A = blind_area(S1, d0s(i), phi0);
            if A < bestA, bestA = A;  d0 = d0s(i); end
        end
    end

    function A = blind_area(S1, d, phi0)
        S2 = S1 + d*[cosd(phi0), sind(phi0)];
        Nr = 200;  Nt = 360;
        rs = linspace(0, cf.R_arena, Nr)';
        ts = linspace(0, 360, Nt+1);  ts(end) = [];
        [RR, TT] = meshgrid(rs, ts);
        px = S1(1) + RR .* cosd(TT);  py = S1(2) + RR .* sind(TT);
        d1 = (px - S1(1)).^2 + (py - S1(2)).^2;
        d2 = (px - S2(1)).^2 + (py - S2(2)).^2;
        cov = (d1 <= cf.R_sense^2) | (d2 <= cf.R_sense^2);
        dr = cf.R_arena/(Nr-1);  dt = 2*pi/Nt;
        A = sum(sum((~cov) .* RR)) * dr * dt;
    end

    % -------- 无响应扇形 Theta_blank (测向角间隙, 含回绕) --------
    function bs = compute_blank_sectors(ths)
        bs = struct('start',{},'end',{},'width',{},'bisector',{});
        ths = sort(mod(ths(:)', 360));  n = numel(ths);
        if n == 0
            bs(1) = struct('start',0,'end',360,'width',360,'bisector',180);  return;
        elseif n == 1
            bs(1) = struct('start',ths(1),'end',ths(1),'width',360,'bisector',mod(ths(1)+180,360));  return;
        end
        for i = 1:n
            a = ths(i);  b = ths(mod(i,n)+1);  w = mod(b-a, 360);
            if w >= cf.min_blank_width
                bs(end+1) = struct('start',a,'end',b,'width',w,'bisector',mod(a+w/2,360)); %#ok<AGROW>
            end
        end
    end

    % -------- 分支 B: 连片未知区域几何质心 -> 虚拟节点 V_explore --------
    function V = generate_virtual_explore_node()
        N = 120;
        xs = linspace(-cf.R_arena, cf.R_arena, N);
        ys = linspace(-cf.R_arena, cf.R_arena, N);
        [XX, YY] = meshgrid(xs, ys);
        inside = (XX.^2 + YY.^2) <= cf.R_arena^2;
        covered = false(size(inside));
        for s = 1:size(st.scans,1)
            covered = covered | ((XX - st.scans(s,1)).^2 + (YY - st.scans(s,2)).^2) <= cf.R_sense^2;
        end
        unknown = inside & ~covered;
        if ~any(unknown(:)), V = []; return; end

        visited = false(size(unknown));
        bestIdx = [];
        for i0 = 1:N
            for j0 = 1:N
                if unknown(i0,j0) && ~visited(i0,j0)
                    si = i0; sj = j0; head = 1; visited(i0,j0) = true;
                    compI = zeros(0,1); compJ = zeros(0,1);
                    while head <= numel(si)
                        i = si(head); j = sj(head); head = head + 1;
                        compI(end+1,1) = i; compJ(end+1,1) = j; %#ok<AGROW>
                        for di = -1:1
                            for dj = -1:1
                                if di == 0 && dj == 0, continue; end
                                ni = i + di; nj = j + dj;
                                if ni >= 1 && ni <= N && nj >= 1 && nj <= N ...
                                   && unknown(ni,nj) && ~visited(ni,nj)
                                    visited(ni,nj) = true;
                                    si(end+1,1) = ni; sj(end+1,1) = nj; %#ok<AGROW>
                                end
                            end
                        end
                    end
                    if numel(compI) > numel(bestIdx)
                        bestIdx = sub2ind(size(XX), compI, compJ);
                    end
                end
            end
        end
        V = [mean(XX(bestIdx)), mean(YY(bestIdx))];
    end

    % -------- 核心函数: plan_greedy_dtsp(贪心最近质心或虚拟节点) --------
    function [k, isv, dest] = plan_greedy_dtsp(V)
        k = [];  isv = false;  dest = [];  best = inf;  cur = [st.x st.y];
        for ch = 1:cf.N_ch
            if T(ch).detected && ~T(ch).is_cleared && isfinite(T(ch).centroid(1))
                if T(ch).radius_Rk > cf.R_conv    % 仅待细化目标作为"去质心"候选
                    d = norm(cur - T(ch).centroid);
                    if d < best, best = d;  k = ch;  isv = false;  dest = T(ch).centroid; end
                end
            end
        end
        if ~isempty(V)
            dv = norm(cur - V);
            if dv < best, k = [];  isv = true;  dest = V; end
        end
    end

    % -------- 核心函数: get_projection_point(线段 AB 到质心 G 的切点) --------
    function [P_cut, dist] = get_projection_point(A, B, G)
        AB = B - A;  L = norm(AB);
        if L < 1e-9, P_cut = A;  dist = norm(A - G);  return; end
        u = dot(G - A, AB)/(L*L);  u = max(0, min(1, u));
        P_cut = A + u*AB;  dist = norm(P_cut - G);
    end

    % -------- 终局切线收敛: 停用 measure, 切线顺路清除, 达 N_min 即退出 --------
    function terminal_tangent_clear(hfig)
        while st.nc < cf.N_min
            best = inf;  kbest = 0;
            for ch = 1:cf.N_ch
                if T(ch).detected && ~T(ch).is_cleared && isfinite(T(ch).centroid(1)) ...
                   && T(ch).radius_Rk <= cf.R_conv
                    d = norm([st.x st.y] - T(ch).centroid);
                    if d < best, best = d;  kbest = ch; end
                end
            end
            if kbest == 0, break; end
            c = T(kbest).centroid;
            r_stop = cf.R_clear - T(kbest).radius_Rk;    % 切点: 距质心 20-R_k, 不落质心
            u = [st.x st.y] - c;  un = norm(u);
            if un < 1e-9, u = [1 0]; else, u = u/un; end
            if un <= r_stop, Pt = [st.x st.y]; else, Pt = c + r_stop*u; end
            act_clear(Pt(1), Pt(2), kbest);
            plot_engine(hfig);
        end
    end

    % -------- 终局早停: 所有活跃目标 R_k<=20m 且 nc>=N_min --------
    function y = check_early_exit()
        act = find([T.detected] & ~[T.is_cleared]);
        if isempty(act)
            y = (st.nc >= cf.N_min);
        else
            y = (st.nc >= cf.N_min) && all([T(act).radius_Rk] <= cf.R_conv);
        end
    end

    % -------- 实时可视化 Plot Engine --------
    function plot_engine(hfig)
        if ~cf.animate || isempty(hfig) || ~ishandle(hfig), return; end
        figure(hfig);  cla;  hold on;  axis equal;  grid on;
        thc = linspace(0, 360, 360);
        plot(cf.R_arena*cosd(thc), cf.R_arena*sind(thc), 'k--', 'LineWidth', 1.2);
        if size(st.path,1) > 1
            plot(st.path(:,1), st.path(:,2), 'b-', 'LineWidth', 1.0);
        end
        plot(st.x, st.y, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 9);
        for ch = 1:cf.N_ch
            if T(ch).detected && ~T(ch).is_cleared && ~isempty(T(ch).convex_hull)
                P = T(ch).convex_hull;
                if T(ch).radius_Rk > cf.R_conv
                    fill(P(:,1), P(:,2), 'b', 'FaceAlpha', 0.12, 'EdgeColor', 'b');
                else
                    fill(P(:,1), P(:,2), 'g', 'FaceAlpha', 0.22, 'EdgeColor', 'g');
                end
                if isfinite(T(ch).centroid(1))
                    cx = T(ch).centroid(1);  cy = T(ch).centroid(2);
                    plot(cx, cy, 'k+', 'MarkerSize', 9);
                    if isfinite(T(ch).radius_Rk) && T(ch).radius_Rk <= cf.R_arena*2
                        plot(cx + T(ch).radius_Rk*cosd(thc), cy + T(ch).radius_Rk*sind(thc), 'm:', 'LineWidth', 1.0);
                    end
                end
            end
        end
        if ~isempty(st.virt)
            plot(st.virt(1), st.virt(2), 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 11);
            text(st.virt(1)+30, st.virt(2)+30, 'V_{explore}', 'Color', 'r', 'FontSize', 9);
        end
        if ~isempty(st.cleared)
            plot(st.cleared(:,1), st.cleared(:,2), 'rx', 'MarkerSize', 13, 'LineWidth', 2.2);
        end
        xlim([-cf.R_arena cf.R_arena]*1.15);  ylim([-cf.R_arena cf.R_arena]*1.15);
        title(sprintf('DTSP: Cleared=%d / N_{min}=%d, vt=%.1fs', st.nc, cf.N_min, st.vt));
        xlabel('x (m)');  ylabel('y (m)');
        drawnow;
    end

end
