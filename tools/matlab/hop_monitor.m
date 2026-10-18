function hop_monitor(port, logfile)
%HOP_MONITOR Live time vs channel plot from a board on USB.
%   HOP_MONITOR finds the Nucleo's serial port, plots the hop log live
%   (last 20 s, refreshed twice a second) and saves everything it reads.
%   HOP_MONITOR(port) uses a given port, e.g. "/dev/cu.usbmodem1403" or "COM5".
%   HOP_MONITOR(port, logfile) also chooses the capture file name.
%   Close the figure to stop. Re-plot a saved capture later with:
%     hop_plot(read_hop_log('hop_capture_....txt'))
%
%   The board must run the bridge firmware (it prints HOP/TX/RX lines).
%   Close any other serial monitor first: only one program can open the port.

    if nargin < 1 || strlength(port) == 0
        ports = serialportlist("available");
        port = ports(contains(ports, "usbmodem") | startsWith(ports, "COM"));
        if isempty(port)
            error('No Nucleo serial port found. Is it plugged in, and is "make monitor" closed?');
        end
        port = port(1);
    end
    if nargin < 2
        logfile = sprintf('hop_capture_%s.txt', datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    end

    s = serialport(port, 115200);
    configureTerminator(s, "CR/LF");
    cleanup = onCleanup(@() delete(s));
    fid = fopen(logfile, 'w');
    closer = onCleanup(@() fclose(fid));
    fprintf('Reading %s, saving to %s. Close the figure to stop.\n', port, logfile);

    fig = figure('Name', "FHSS hop monitor: " + port);
    ax = axes(fig);
    lines = strings(0, 1);
    next_draw = tic;
    while isvalid(fig)
        while s.NumBytesAvailable > 0
            line = strtrim(readline(s));
            fprintf(fid, '%s\n', line);
            if startsWith(line, ["CFG,", "HOP,", "TX,", "RX,"])
                lines(end + 1, 1) = line; %#ok<AGROW>
            else
                fprintf('%s\n', line);    % status and message lines to the Command Window
            end
        end
        if toc(next_draw) > 0.5 && ~isempty(lines)
            next_draw = tic;
            hop_plot(read_hop_log(lines), ax, 20);
            drawnow limitrate;
        else
            pause(0.02);
        end
    end
end
