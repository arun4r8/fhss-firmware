%TEST_FHSS_SEQUENCE Check fhss_sequence.m against the firmware's own output.
%   Run from tools/matlab:  test_fhss_sequence
%   ref_sequences.csv comes from gen_ref_sequences.c, which calls the real
%   src/fhss.c, so a pass means MATLAB and the boards hop identically.

% read line by line: rows have different lengths, and readmatrix silently
% drops the shorter ones as if they were headers
lines = readlines(fullfile(fileparts(mfilename('fullpath')), 'ref_sequences.csv'));
lines = lines(strlength(lines) > 0);
fails = 0;
for r = 1:numel(lines)
    row = str2double(split(lines(r), ','))';
    seed = uint32(row(1));
    n = row(2);
    want = row(3:2 + n);
    got = fhss_sequence(seed, n);
    if ~isequal(got, want)
        fails = fails + 1;
        k = find(got ~= want, 1);
        fprintf('FAIL seed %u, %d channels: first difference at hop %d (got %d, want %d)\n', ...
                seed, n, k - 1, got(k), want(k));
    end
    if ~isequal(sort(got), 0:n - 1)
        fails = fails + 1;
        fprintf('FAIL seed %u, %d channels: not every channel exactly once\n', seed, n);
    end
end

% the pair of seeds on the boards should look nothing alike
a = fhss_sequence(0x6A5D10);
b = fhss_sequence(0x6A5D11);
if mean(a == b) > 0.2
    fails = fails + 1;
    fprintf('FAIL neighbouring seeds give similar sequences\n');
end

fprintf('%d sequences checked against the firmware, %d failures\n', numel(lines), fails);
if fails
    error('fhss_sequence does not match the firmware');
end
