#include "pod_cmd.h"

size_t pod_cmd_build(uint8_t state, uint16_t seq, uint8_t *out, size_t max)
{
    if (max < POD_CMD_LEN) {
        return 0;
    }
    out[0] = POD_CMD_MAGIC;
    out[1] = (uint8_t)((state & 0x0Fu) << 4);
    out[2] = (uint8_t)(seq >> 8);
    out[3] = (uint8_t)seq;
    return POD_CMD_LEN;
}

bool pod_cmd_parse(const uint8_t *in, size_t len, uint8_t *state, uint16_t *seq)
{
    if (len != POD_CMD_LEN || in[0] != POD_CMD_MAGIC) {
        return false;
    }
    *state = in[1] >> 4;
    *seq = (uint16_t)((in[2] << 8) | in[3]);
    return true;
}
