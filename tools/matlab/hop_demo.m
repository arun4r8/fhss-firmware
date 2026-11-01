function result = hop_demo(duration_s, ports)
%HOP_DEMO Live demo: both boards' hopping and packets on one plot.
%   HOP_DEMO plots 15 s of the pod (leader) and ground (follower) boards
%   live: every channel change, every packet sent and every packet
%   received. Events are placed by the boards' own millisecond clocks (the
%   ground's clock is lined up to the pod's by matching hop numbers), so
%   USB and plotting delays don't smear the picture.
%   HOP_DEMO(duration_s) runs for another length of time.
%   HOP_DEMO(duration_s, ports) uses given serial ports, e.g.
%     ["/dev/cu.usbmodem11303" "/dev/cu.usbmodem11403"] or ["COM5" "COM6"].
%   result = HOP_DEMO(...) also returns the counts.
%
%   Needs both boards on USB running the bridge firmware (any role) and no
%   other program holding their serial ports. Which board is which is
%   worked out from what it prints (the pod logs TX lines, the ground RX).
%   Saves the final plot as hop_demo_<date>.png.

    if nargin < 1 || isempty(duration_s)
        duration_s = 15;
    end
    if nargin < 2 || isempty(ports)
        ports = serialportlist("available");
        % macOS lists each board twice (tty.* and cu.*, the same port): keep cu.*
        ports = ports(startsWith(ports, "/dev/cu.usbmodem") | startsWith(ports, "COM"));
    end
    if numel(ports) < 1
        error('No board serial ports found. Are the boards plugged in, and other serial monitors closed?');
    end

    s = cell(1, numel(ports));
    for k = 1:numel(ports)
        s{k} = serialport(ports(k), 115200);
        configureTerminator(s{k}, "CR/LF");
    end
    cleanup = onCleanup(@() cellfun(@delete, s));

    % ---- figure ----------------------------------------------------------
    visible = 'on';
    if ~usejava('desktop'), visible = 'off'; end   % batch runs: draw off screen
    fig = figure('Name', 'FHSS live demo', 'Color', 'w', 'Position', [80 80 1300 620], ...
                 'Visible', visible);
    ax = axes(fig);
    hold(ax, 'on');
    grid(ax, 'on');
    ax.Color = 'w';
    ax.XColor = [0.15 0.15 0.15];
    ax.YColor = [0.15 0.15 0.15];
    ax.GridColor = [0.6 0.6 0.6];
    xlim(ax, [0 duration_s]);
    ylim(ax, [-1 50]);
    xlabel(ax, 'Time (s)');
    ylabel(ax, 'Channel  (902.2 + 0.5 \times ch MHz)');
    h_hop = plot(ax, NaN, NaN, '-', 'Color', [0.65 0.65 0.65], 'LineWidth', 2, ...
                 'DisplayName', 'pod channel (hopping every 400 ms)');
    h_tx = plot(ax, NaN, NaN, '-', 'Color', [0.1 0.35 0.9], 'LineWidth', 7, ...
                'DisplayName', 'pod transmits');
    h_rx = plot(ax, NaN, NaN, 'v', 'Color', [0.85 0.15 0.15], 'MarkerFaceColor', [0.85 0.15 0.15], ...
                'MarkerSize', 8, 'DisplayName', 'ground receives');
    h_gnd = plot(ax, NaN, NaN, 'o', 'Color', [0.95 0.55 0.1], 'MarkerSize', 5, 'LineWidth', 1.2, ...
                 'DisplayName', 'ground follows the hop');
    legend(ax, 'Location', 'eastoutside', 'TextColor', [0.15 0.15 0.15], 'Color', 'w');
    ttl = title(ax, 'Waiting for the boards...', 'Color', [0.1 0.1 0.1], 'Interpreter', 'none');
    sub = subtitle(ax, '', 'Color', [0.35 0.35 0.35], 'Interpreter', 'none');

    % ---- live loop -------------------------------------------------------
    role = strings(1, numel(ports));         % "pod" / "ground", learned from the lines
    % all times are board milliseconds: pod clock for pod events, ground
    % clock for ground events until converted with the hop-matched offset
    hop_t = []; hop_ch = []; hop_i = [];     % pod channel changes (+ hop index)
    tx_t = []; tx_ch = []; tx_air = [];      % pod transmissions (start, channel, airtime s)
    rx_t = []; rx_ch = [];                   % ground receptions
    g_t = []; g_ch = []; g_i = [];           % ground channel changes while locked
    pod0 = NaN;                              % pod ms that maps to t = 0
    last_msg = "";
    cellfun(@flush, s);                      % start from now, not from buffered lines
    t0 = tic;
    next_draw = 0;
    while (isnan(pod0) || isempty(hop_t) || (max([hop_t tx_t]) - pod0) / 1000 < duration_s) ...
            && toc(t0) < duration_s + 10 && isvalid(fig)
        for k = 1:numel(ports)
            while s{k}.NumBytesAvailable > 0
                line = strtrim(readline(s{k}));
                f = split(line, ',');
                v = str2double(f(2:end))';
                switch f(1)
                    case "TX"
                        role(k) = "pod";
                        if numel(f) == 5 && all(isfinite(v))
                            if isnan(pod0), pod0 = v(1); end
                            tx_t(end + 1) = v(1); tx_ch(end + 1) = v(3); %#ok<AGROW>
                            tx_air(end + 1) = lora_airtime_ms(v(4) + 6) / 1000; %#ok<AGROW>
                        end
                    case "RX"
                        role(k) = "ground";
                        if numel(f) == 4 && all(isfinite(v))
                            rx_t(end + 1) = v(1); rx_ch(end + 1) = v(3); %#ok<AGROW>
                        end
                    case "HOP"
                        if numel(f) == 5 && all(isfinite(v(1:3))) && role(k) == "pod"
                            if isnan(pod0), pod0 = v(1); end
                            hop_t(end + 1) = v(1); hop_ch(end + 1) = v(3); hop_i(end + 1) = v(2); %#ok<AGROW>
                        elseif numel(f) == 5 && all(isfinite(v(1:3))) && role(k) == "ground" && f(5) == "H"
                            g_t(end + 1) = v(1); g_ch(end + 1) = v(3); g_i(end + 1) = v(2); %#ok<AGROW>
                        end
                    otherwise
                        if startsWith(line, "rx hub")
                            last_msg = line;
                        end
                end
            end
        end
        if toc(t0) >= next_draw
            next_draw = toc(t0) + 0.1;
            redraw();
            drawnow limitrate;
        else
            pause(0.005);
        end
    end
    redraw();
    drawnow;

    % ---- summary ---------------------------------------------------------
    delivered = numel(rx_t);
    sent = numel(tx_t);
    result = struct('hops', numel(hop_t), 'sent', sent, 'received', delivered, ...
                    'delivery_pct', 100 * delivered / max(sent, 1), ...
                    'channels_used', numel(unique(hop_ch)));
    fprintf('%d s: %d hops over %d channels, pod sent %d packets, ground received %d (%.0f%%)\n', ...
            duration_s, result.hops, result.channels_used, sent, delivered, result.delivery_pct);
    if ~any(role == "pod"),    fprintf('  (no pod seen: is it plugged in and running the bridge firmware?)\n'); end
    if ~any(role == "ground"), fprintf('  (no ground seen: is it locked? It needs up to 20 s after the pod starts)\n'); end
    out = sprintf('hop_demo_%s.png', datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    exportgraphics(fig, out, 'BackgroundColor', 'white');
    fprintf('  plot saved to %s\n', out);

    function redraw()
        if isnan(pod0)
            return
        end
        sec = @(ms) (ms - pod0) / 1000;      % pod board ms -> plot seconds
        if ~isempty(hop_t)
            ends = [hop_t(2:end) max([hop_t(end) tx_t]) + 1];   % each hop held until the next
            x = [sec(hop_t); sec(ends)];
            y = [hop_ch; hop_ch];
            set(h_hop, 'XData', x(:), 'YData', y(:));
        end
        if ~isempty(tx_t)
            x = [sec(tx_t); sec(tx_t) + tx_air; nan(size(tx_t))];
            y = [tx_ch; tx_ch; nan(size(tx_ch))];
            set(h_tx, 'XData', x(:), 'YData', y(:));
        end
        % ground clock -> pod clock: the same hop index starts at the same
        % moment on both boards once locked (within a few ms)
        % A hop index repeats only every 50 hops (20 s), so for each recent
        % ground hop take the pod hop with the same index nearest in time.
        off = NaN;
        d = [];
        for q = max(1, numel(g_t) - 20):numel(g_t)
            m = find(hop_i == g_i(q));
            if isempty(m), continue; end
            [~, j] = max(hop_t(m));               % the latest such pod hop
            d(end + 1) = g_t(q) - hop_t(m(j)); %#ok<AGROW>
        end
        if ~isempty(d)
            off = median(d);
        end
        if isfinite(off)
            set(h_rx, 'XData', sec(rx_t - off), 'YData', rx_ch);
            set(h_gnd, 'XData', sec(g_t - off), 'YData', g_ch);
        end
        ttl.String = sprintf('Frequency hopping, live: %d hops, %d channels, pod sent %d, ground received %d (%.0f%%)', ...
                             numel(hop_t), numel(unique(hop_ch)), numel(tx_t), numel(rx_t), ...
                             100 * numel(rx_t) / max(numel(tx_t), 1));
        if strlength(last_msg) > 0
            sub.String = "latest telemetry at the ground: " + extractBefore(last_msg + "                                                            ", 60);
        end
    end
end
