function ms = lora_airtime_ms(payload_bytes, sf, bw_hz, cr, preamble)
%LORA_AIRTIME_MS Time on air of one LoRa packet, same formula as the firmware
%   (sx1262_airtime_ms in ports/nucleo_sx1262/sx1262.c): explicit header,
%   CRC on. payload_bytes is the whole radio payload, including the link's
%   6-byte header. Defaults are the link's settings: SF7, 125 kHz, CR 4/5,
%   8-symbol preamble.
%
%   lora_airtime_ms(37)   % full telemetry packet -> 83 ms

    if nargin < 2, sf = 7; end
    if nargin < 3, bw_hz = 125000; end
    if nargin < 4, cr = 1; end            % 1..4 means 4/5..4/8
    if nargin < 5, preamble = 8; end

    sym_us = floor(2^sf * 1e6 / bw_hz);
    de = sym_us > 16000;                  % low data rate optimization
    num = 8 * payload_bytes - 4 * sf + 28 + 16;
    den = 4 * (sf - 2 * de);
    blocks = max(ceil(num / den), 0);
    payload_syms = 8 + blocks * (cr + 4);
    quarter_syms = 4 * (preamble + payload_syms) + 17;   % preamble + 4.25 sync symbols
    ms = ceil(floor(quarter_syms * sym_us / 4) / 1000);
end
