%TEST_HOP_TOOLS Check read_hop_log and hop_plot on a simulated capture.
%   Run from tools/matlab:  test_hop_tools
%   Builds a 60 s capture in the firmware's log format (with console noise,
%   a line cut in half, one hop on the wrong channel and one transmission
%   that runs past a hop) and checks the parser and the checks catch them.
%   Saves the plot as test_hop_plot.png to look at.

seed = 0x6A5D10;
L = make_log(seed, true);
tmp = [tempname '.txt'];
writelines(L, tmp);
log = read_hop_log(tmp);
delete(tmp);

fails = 0;
check = @(ok, msg) ok || fprintf('FAIL %s\n', msg) < 0;
fails = fails + ~check(height(log.hops) == 150, sprintf('parsed %d hops, want 150', height(log.hops)));
fails = fails + ~check(height(log.tx) == 150, sprintf('parsed %d TX, want 150', height(log.tx)));
fails = fails + ~check(log.cfg.seed == seed && log.cfg.role == "pod", 'CFG line not parsed');

fig = figure('Visible', 'off', 'Position', [0 0 1400 600]);
r = hop_plot(log, axes(fig));
fails = fails + ~check(r.mismatches == 1, sprintf('%d mismatches, want 1', r.mismatches));
fails = fails + ~check(r.tx_straddles == 1, sprintf('%d straddles, want 1', r.tx_straddles));
fails = fails + ~check(all(r.dwell_ms >= 398 & r.dwell_ms <= 402), sprintf('dwell %g-%g', r.dwell_ms));
fails = fails + ~check(r.min_revisit_s < 20, 'off-sequence hop did not shorten the revisit time');
exportgraphics(fig, 'test_hop_plot.png');
close(fig);

% a clean capture must pass every check exactly
clean = read_hop_log(make_log(seed, false));
fig = figure('Visible', 'off');
r = hop_plot(clean, axes(fig));
close(fig);
fails = fails + ~check(r.mismatches == 0 && r.tx_straddles == 0, 'clean capture flagged');
fails = fails + ~check(abs(r.min_revisit_s - 20) <= 0.003, sprintf('clean revisit %.3f s, want 20', r.min_revisit_s));
fails = fails + ~check(all(r.channel_uses == 3), 'clean capture: channels not used equally');

% the same lines as the live monitor passes them: one at a time
one = read_hop_log(L(2));
fails = fails + ~check(one.cfg.seed == seed, 'single line not parsed');

fprintf('hop tools: %d failures (plot saved to test_hop_plot.png)\n', fails);
if fails
    error('hop tools test failed');
end

function L = make_log(seed, with_faults)
    % 150 hops (60 s) in the firmware's format, with console noise and a
    % half line; with_faults adds an off-sequence hop and a late packet
    n = 50;
    seq = fhss_sequence(seed, n);
    L = ["status: CAN frames 0, hub msgs 0, sent 0 (0 pkts)"
         sprintf("CFG,pod,%d,50,902200000,500000,400", seed)];
    t = 1234;                          % board uptime when hopping starts
    for k = 0:149
        ch = seq(mod(k, n) + 1);
        if with_faults && k == 77
            ch = mod(ch + 1, n);       % off-sequence hop
        end
        L(end + 1) = sprintf('HOP,%d,%d,%d,H', t + k * 400 + mod(k, 3), mod(k, n), ch); %#ok<AGROW>
        tx_at = t + k * 400 + 10;
        if with_faults && k == 100
            tx_at = t + k * 400 + 380; % 83 ms packet starting 20 ms before the hop
        end
        L(end + 1) = sprintf('TX,%d,%d,%d,31', tx_at, mod(k, n), ch); %#ok<AGROW>
    end
    L(end + 1) = "HOP,61234,15";      % line cut off mid-way
    L(end + 1) = "rx hub 3, rssi -48 dBm: 0=1234";
end
