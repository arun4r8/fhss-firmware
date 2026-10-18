function seq = fhss_sequence(seed, num_channels)
%FHSS_SEQUENCE Hop sequence the firmware builds from a seed (src/fhss.c).
%   seq = FHSS_SEQUENCE(seed) returns the 50-channel order for the given
%   seed, as channel numbers 0..49 in hop order. seq(k) is the channel used
%   on hop index k-1 (MATLAB indexes from 1, the firmware from 0).
%
%   seq = FHSS_SEQUENCE(seed, num_channels) for other channel counts.
%
%   Must match the firmware bit for bit, so every step copies the C code:
%   a murmur3 finalizer mixes the seed, xorshift32 generates random numbers,
%   rejection sampling makes them unbiased, and Fisher-Yates shuffles.
%
%   Example (the seed both boards use):
%     seq = fhss_sequence(0x6A5D10);
%     freq_mhz = 902.2 + 0.5 * seq;   % channel -> frequency

    if nargin < 2
        num_channels = 50;
    end
    s = mix_seed(seed);
    seq = uint8(0:num_channels - 1);
    for i = num_channels - 1:-1:1          % C: for (i = n - 1; i > 0; i--)
        [j, s] = rand_below(s, i + 1);
        tmp = seq(i + 1);                  % +1: C indexes from 0
        seq(i + 1) = seq(j + 1);
        seq(j + 1) = tmp;
    end
    seq = double(seq);
end

function x = mix_seed(x)
    % murmur3 finalizer so nearby seeds give unrelated sequences
    x = uint32(x);
    x = bitxor(x, bitshift(x, -16));       % x ^= x >> 16
    x = mul32(x, 0x85EBCA6Bu32);           % x *= 0x85ebca6b
    x = bitxor(x, bitshift(x, -13));       % x ^= x >> 13
    x = mul32(x, 0xC2B2AE35u32);           % x *= 0xc2b2ae35
    x = bitxor(x, bitshift(x, -16));       % x ^= x >> 16
    if x == 0
        x = 0x6D2B79F5u32;                 % xorshift state must be nonzero
    end
end

function [x, s] = xorshift32(s)
    % MATLAB passes by value, so the new state is returned (C: *s = x)
    x = s;
    x = bitxor(x, bitshift(x, 13));        % x ^= x << 13 (bits off the top drop, as in C)
    x = bitxor(x, bitshift(x, -17));       % x ^= x >> 17
    x = bitxor(x, bitshift(x, 5));         % x ^= x << 5
    s = x;
end

function [r, s] = rand_below(s, bound)
    % uniform integer in [0, bound) without modulo bias
    bound = uint32(bound);
    limit = intmax('uint32') - mod(intmax('uint32'), bound);
    while true
        [r, s] = xorshift32(s);
        if r < limit
            break
        end
    end
    r = double(mod(r, bound));
end

function p = mul32(a, b)
    % 32-bit multiply that wraps like C. MATLAB integers saturate instead,
    % so multiply in 64 bits (the product always fits) and keep the low 32.
    p = uint32(bitand(uint64(a) * uint64(b), 0xFFFFFFFFu64));
end
