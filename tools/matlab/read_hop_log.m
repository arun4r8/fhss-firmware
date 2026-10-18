function log = read_hop_log(source)
%READ_HOP_LOG Parse the bridge firmware's log lines (boards/nucleo_wl55jc/bridge.c).
%   log = READ_HOP_LOG('capture.txt') reads a saved serial capture.
%   log = READ_HOP_LOG(lines) parses a string array of lines (live use).
%
%   Returns a struct:
%     log.cfg   role, seed, channels, base_hz, spacing_hz, dwell_ms
%               (from the CFG line; defaults to the boards' settings if the
%               capture started after it)
%     log.hops  table: ms, hop, channel, locked   (one row per channel change)
%     log.tx    table: ms, hop, channel, bytes    (transmission start times)
%     log.rx    table: ms, hop, channel           (packets received)
%   Other console lines (status, messages) are ignored.

    if (ischar(source) || (isstring(source) && isscalar(source))) && isfile(source)
        lines = readlines(source);
    else
        lines = string(source);   % already lines (string array or cell array)
    end
    lines = strtrim(lines(:));

    log.cfg = struct('role', "unknown", 'seed', 0x6A5D10, 'channels', 50, ...
                     'base_hz', 902200000, 'spacing_hz', 500000, 'dwell_ms', 400);
    hop = zeros(0, 4);
    tx = zeros(0, 4);
    rx = zeros(0, 3);

    for k = 1:numel(lines)
        f = split(lines(k), ',');
        switch f(1)
            case "CFG"
                if numel(f) == 7
                    v = str2double(f(3:7));
                    if all(isfinite(v))
                        log.cfg = struct('role', f(2), 'seed', v(1), 'channels', v(2), ...
                                         'base_hz', v(3), 'spacing_hz', v(4), 'dwell_ms', v(5));
                    end
                end
            case "HOP"
                if numel(f) == 5
                    v = [str2double(f(2:4))', f(5) == "H"];
                    if all(isfinite(v)), hop(end + 1, :) = v; end %#ok<AGROW>
                end
            case "TX"
                if numel(f) == 5
                    v = str2double(f(2:5))';
                    if all(isfinite(v)), tx(end + 1, :) = v; end %#ok<AGROW>
                end
            case "RX"
                if numel(f) == 4
                    v = str2double(f(2:4))';
                    if all(isfinite(v)), rx(end + 1, :) = v; end %#ok<AGROW>
                end
        end
    end
    % a line cut off mid-way (capture started late) fails the field count or
    % the number parse above and is skipped

    log.hops = array2table(hop, 'VariableNames', {'ms', 'hop', 'channel', 'locked'});
    log.hops.locked = logical(log.hops.locked);
    log.tx = array2table(tx, 'VariableNames', {'ms', 'hop', 'channel', 'bytes'});
    log.rx = array2table(rx, 'VariableNames', {'ms', 'hop', 'channel'});
end
