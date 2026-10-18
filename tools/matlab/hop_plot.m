function result = hop_plot(log, ax, window_s)
%HOP_PLOT Time vs channel plot of a hop log, with the predicted sequence.
%   result = HOP_PLOT(log) plots a log from read_hop_log in a new figure.
%   HOP_PLOT(log, ax) draws into existing axes (hop_monitor uses this).
%   HOP_PLOT(log, ax, window_s) shows only the last window_s seconds.
%
%   Plot: x = time (s, board clock), y = channel 0..N-1.
%     grey steps   where fhss_sequence(seed) says the board should be
%     dots         channel changes the board logged (hollow = searching)
%     blue bars    transmissions, drawn for their real time on air
%     red marks    packets received
%     red rings    logged hops on the wrong channel, and transmissions that
%                  ran past the next hop (both should never happen)
%
%   result: struct with the checks
%     mismatches     logged hops on a different channel than predicted
%     dwell_ms       [min max] time between consecutive hops while hopping
%     min_revisit_s  shortest time before any channel was used again
%     channel_uses   visits per channel (equal use -> all about the same)
%     tx_straddles   transmissions that ran past the next hop

    if nargin < 2 || isempty(ax)
        figure('Name', 'FHSS hop log');
        ax = axes;
    end
    if nargin < 3
        window_s = inf;
    end
    c = log.cfg;
    n = c.channels;
    seq = fhss_sequence(uint32(c.seed), n);
    h = log.hops;
    hop_rows = h(h.locked, :);

    % ---- checks ---------------------------------------------------------
    result = struct('mismatches', 0, 'dwell_ms', [NaN NaN], 'min_revisit_s', NaN, ...
                    'channel_uses', zeros(1, n), 'tx_straddles', 0);
    wrong = false(height(hop_rows), 1);
    if ~isempty(hop_rows)
        predicted = seq(mod(hop_rows.hop, n) + 1)';
        wrong = predicted ~= hop_rows.channel;
        result.mismatches = sum(wrong);
        consecutive = diff(hop_rows.hop) == 1 | diff(hop_rows.hop) == 1 - n;
        d = diff(hop_rows.ms);
        if any(consecutive)
            result.dwell_ms = [min(d(consecutive)) max(d(consecutive))];
        end
        result.channel_uses = accumarray(hop_rows.channel + 1, 1, [n 1])';
        gaps = [];
        for ch = 0:n - 1
            t = hop_rows.ms(hop_rows.channel == ch);
            gaps = [gaps; diff(t)]; %#ok<AGROW>
        end
        if ~isempty(gaps)
            result.min_revisit_s = min(gaps) / 1000;
        end
    end
    late = false(height(log.tx), 1);
    for k = 1:height(log.tx)
        next_hop = h.ms(find(h.ms > log.tx.ms(k), 1));
        air = lora_airtime_ms(log.tx.bytes(k) + 6);
        late(k) = ~isempty(next_hop) && log.tx.ms(k) + air > next_hop;
    end
    result.tx_straddles = sum(late);

    % ---- plot -----------------------------------------------------------
    cla(ax);
    hold(ax, 'on');
    t_end = max([h.ms; log.tx.ms; log.rx.ms; 0]) / 1000;
    t_start = max(min([h.ms; inf]) / 1000, t_end - window_s);
    if ~isfinite(t_start), t_start = 0; end

    if ~isempty(hop_rows)
        % predicted track from the first hop while hopping, one step per dwell
        t0 = hop_rows.ms(1);
        k = 0:ceil((t_end * 1000 - t0) / c.dwell_ms);
        tp = (t0 + k * c.dwell_ms) / 1000;
        chp = seq(mod(hop_rows.hop(1) + k, n) + 1);
        keep = tp >= t_start - c.dwell_ms / 1000;
        stairs(ax, [tp(keep) tp(end) + c.dwell_ms / 1000], [chp(keep) chp(end)], ...
               'Color', [0.7 0.7 0.7], 'LineWidth', 2, 'DisplayName', 'predicted (seed)');
    end
    in = @(t) t / 1000 >= t_start;
    hl = h(h.locked & in(h.ms), :);
    hs = h(~h.locked & in(h.ms), :);
    % colors chosen to read on both the light and dark MATLAB themes
    plot(ax, hl.ms / 1000, hl.channel, '.', 'Color', [0.95 0.55 0.1], 'MarkerSize', 16, ...
         'DisplayName', 'logged hop');
    if ~isempty(hs)
        plot(ax, hs.ms / 1000, hs.channel, 'o', 'Color', [0.95 0.55 0.1], ...
             'DisplayName', 'logged (searching)');
    end
    tx = log.tx(in(log.tx.ms), :);
    if ~isempty(tx)
        air = arrayfun(@(b) lora_airtime_ms(b + 6), tx.bytes);
        x = [tx.ms, tx.ms + air, nan(height(tx), 1)]' / 1000;
        y = repmat(tx.channel', 3, 1);
        plot(ax, x(:), y(:), '-', 'Color', [0.1 0.35 0.9], 'LineWidth', 5, 'DisplayName', 'TX');
    end
    rx = log.rx(in(log.rx.ms), :);
    if ~isempty(rx)
        plot(ax, rx.ms / 1000, rx.channel, 'v', 'Color', [0.85 0.15 0.15], ...
             'MarkerFaceColor', [0.85 0.15 0.15], 'DisplayName', 'RX');
    end
    bad_hops = hop_rows(wrong & in(hop_rows.ms), :);
    bad_tx = log.tx(late & in(log.tx.ms), :);
    if ~isempty(bad_hops) || ~isempty(bad_tx)
        plot(ax, [bad_hops.ms; bad_tx.ms] / 1000, [bad_hops.channel; bad_tx.channel], 'o', ...
             'Color', [0.95 0.15 0.15], 'MarkerSize', 16, 'LineWidth', 2.5, ...
             'DisplayName', 'problem');
    end
    hold(ax, 'off');

    ylim(ax, [-1 n]);
    if t_end > t_start
        xlim(ax, [t_start t_end + 0.2]);
    end
    xlabel(ax, 'Time (s, board clock)');
    ylabel(ax, sprintf('Channel  (%.1f + %.1f \\times ch MHz)', c.base_hz / 1e6, c.spacing_hz / 1e6));
    grid(ax, 'on');
    legend(ax, 'Location', 'eastoutside');
    title(ax, sprintf(['%s, seed 0x%X: %d hops, %d off-sequence, dwell %g-%g ms, ' ...
                       'min revisit %.1f s, %d TX past a hop'], ...
                      c.role, c.seed, height(hop_rows), result.mismatches, result.dwell_ms, ...
                      result.min_revisit_s, result.tx_straddles), 'Interpreter', 'none');
end
