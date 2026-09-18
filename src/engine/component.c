/*
 * component.c — bounded runtime reader for authored engine selection.
 *
 * engsnd.dat maps each VCF ENG NUM to a zero-based ENG COMP ID; the selected
 * compnent.cdf ENGN row supplies the curve. The native chain is documented in
 * docs/specs/re/engine-index-mapping.md. The authored brake/suspension floats
 * deliberately stay unnamed and are not parsed here. Static CDF evidence and
 * the independent reference parser live in
 * docs/specs/re/compnent-cdf-010-editor.md and tools/cdf_010.py.
 */
#include "component.h"
#include "engine/vfs.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CDF_EXPECTED_SIZE 3608u
#define CDF_ENGINE_COUNT 4u
#define CDF_ENGINE_K_SCALE 8.163265619032245e-08f
#define ENGSND_MAX_ROWS 32u
#define ENGSND_MAX_LINE 192u

static const size_t ENGINE_OFFSETS[CDF_ENGINE_COUNT] = {
    0x428u, 0x44au, 0x46cu, 0x48eu
};
static const size_t BRAKE_OFFSETS[CDF_ENGINE_COUNT] = {
    0x8ccu, 0x8eeu, 0x910u, 0x932u
};
static const size_t SUSPENSION_OFFSETS[CDF_ENGINE_COUNT] = {
    0xd70u, 0xd96u, 0xdbcu, 0xde2u
};

static uint32_t rd_u32(const uint8_t *p)
{
    uint32_t v;
    memcpy(&v, p, sizeof v);
    return v;
}

static float rd_f32(const uint8_t *p)
{
    float v;
    memcpy(&v, p, sizeof v);
    return v;
}

static int chunk_is(const uint8_t *buf, size_t size, size_t off,
                    const char tag[4], uint32_t total)
{
    return off <= size && size - off >= 8u &&
           memcmp(buf + off, tag, 4u) == 0 &&
           rd_u32(buf + off + 4u) == total &&
           (size_t)total <= size - off;
}

/* Shipped text format: ENG NUM, ENG COMP ID, REVABLE, then eight sound/time
 * fields. Parse the whole row so a shifted/truncated table cannot silently
 * become a physics mapping. The native parser is FUN_00452ee0. */
static int engine_component_id(uint32_t engine_num, uint32_t *component_id)
{
    size_t size = 0;
    char *buf = vfs_read_file("engsnd.dat", &size);
    if (!buf) return -1;

    uint32_t ids[ENGSND_MAX_ROWS];
    size_t pos = 0, rows = 0;
    int found = 0, ok = 1;
    uint32_t selected = 0;
    while (ok && pos < size) {
        size_t eol = pos;
        while (eol < size && buf[eol] != '\n') eol++;
        size_t len = eol - pos;
        if (len && buf[pos + len - 1u] == '\r') len--;
        if (len >= ENGSND_MAX_LINE) {
            ok = 0;
            break;
        }

        char line[ENGSND_MAX_LINE];
        memcpy(line, buf + pos, len);
        line[len] = '\0';
        pos = eol < size ? eol + 1u : size;

        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '#') continue;

        long long eng = -1, comp = -1;
        int revable = -1;
        char engine_wav[16], horn_wav[16], ign0[16], ign1[16], ign2[16];
        double time0 = 0.0, time1 = 0.0, time2 = 0.0;
        int consumed = -1;
        int got = sscanf(p,
            "%lld %lld %d %15s %15s %15s %lf %15s %lf %15s %lf %n",
            &eng, &comp, &revable, engine_wav, horn_wav,
            ign0, &time0, ign1, &time1, ign2, &time2, &consumed);
        if (consumed >= 0)
            while (p[consumed] == ' ' || p[consumed] == '\t') consumed++;
        if (got != 11 || consumed < 0 || p[consumed] != '\0' ||
            eng < 0 || eng > UINT32_MAX ||
            comp < 0 || comp >= CDF_ENGINE_COUNT ||
            (revable != 0 && revable != 1) ||
            !isfinite(time0) || !isfinite(time1) || !isfinite(time2) ||
            time0 < 0.0 || time1 < 0.0 || time2 < 0.0 ||
            rows >= ENGSND_MAX_ROWS) {
            ok = 0;
            break;
        }
        for (size_t i = 0; i < rows; i++)
            if (ids[i] == (uint32_t)eng) ok = 0;
        if (!ok) break;
        ids[rows++] = (uint32_t)eng;
        if ((uint32_t)eng == engine_num) {
            selected = (uint32_t)comp;
            found = 1;
        }
    }
    vfs_free(buf);
    if (!ok || rows == 0 || !found) return -1;
    *component_id = selected;
    return 0;
}

int component_engine_curve_by_id(uint32_t component_id,
                                 ComponentEngineCurve *out)
{
    if (!out || component_id >= CDF_ENGINE_COUNT) return -1;

    size_t size = 0;
    uint8_t *buf = vfs_read_file("compnent.cdf", &size);
    if (!buf) return -1;

    int ok = size == CDF_EXPECTED_SIZE &&
        chunk_is(buf, size, 0x000u, "BWD2", 8u) &&
        chunk_is(buf, size, 0x008u, "REV\0", 12u) &&
        rd_u32(buf + 0x10u) == 1u &&
        chunk_is(buf, size, 0x014u, "ECNK", 8u) &&
        chunk_is(buf, size, 0x01cu, "NTBL", 1036u) &&
        rd_u32(buf + 0x024u) == CDF_ENGINE_COUNT &&
        chunk_is(buf, size, 0x4b0u, "EXIT", 8u) &&
        chunk_is(buf, size, 0x4b8u, "BCNK", 8u) &&
        chunk_is(buf, size, 0x4c0u, "BTBL", 1036u) &&
        rd_u32(buf + 0x4c8u) == CDF_ENGINE_COUNT &&
        chunk_is(buf, size, 0x954u, "EXIT", 8u) &&
        chunk_is(buf, size, 0x95cu, "SCNK", 8u) &&
        chunk_is(buf, size, 0x964u, "STBL", 1036u) &&
        rd_u32(buf + 0x96cu) == CDF_ENGINE_COUNT &&
        chunk_is(buf, size, 0xe08u, "EXIT", 8u) &&
        chunk_is(buf, size, 0xe10u, "EXIT", 8u);

    /* Validate every record boundary/id/terminal marker before consuming one
     * engine. Brake/suspension payload floats remain deliberately unread. */
    for (uint32_t i = 1u; ok && i <= CDF_ENGINE_COUNT; i++) {
        char engine_id[6] = { 'e', 'n', 'g', '0', (char)('0' + i), '\0' };
        char brake_id[6] = { 'b', 'r', 'a', '0', (char)('0' + i), '\0' };
        char suspension_id[6] = { 's', 'u', 's', '0', (char)('0' + i), '\0' };
        size_t eo = ENGINE_OFFSETS[i - 1u];
        size_t bo = BRAKE_OFFSETS[i - 1u];
        size_t so = SUSPENSION_OFFSETS[i - 1u];
        uint8_t terminal = (uint8_t)(i == CDF_ENGINE_COUNT);
        ok = chunk_is(buf, size, eo, "ENGN", 34u) &&
             memcmp(buf + eo + 20u, engine_id, sizeof engine_id) == 0 &&
             buf[eo + 33u] == terminal &&
             chunk_is(buf, size, bo, "BRAK", 34u) &&
             memcmp(buf + bo + 20u, brake_id, sizeof brake_id) == 0 &&
             buf[bo + 33u] == terminal &&
             chunk_is(buf, size, so, "SUSP", 38u) &&
             memcmp(buf + so + 24u, suspension_id,
                    sizeof suspension_id) == 0 &&
             buf[so + 37u] == terminal;
    }

    if (ok) {
        size_t off = ENGINE_OFFSETS[component_id];
        const uint8_t *p = buf + off + 8u;
        float tpeak = rd_f32(p + 4u);
        float catalog_value = rd_f32(p + 8u);
        float k = tpeak * CDF_ENGINE_K_SCALE;
        ok = rd_u32(p) > 0u && isfinite(tpeak) && tpeak > 0.0f &&
             isfinite(catalog_value) && catalog_value > 0.0f &&
             isfinite(k) && k > 0.0f;
        if (ok) {
            out->component_id = component_id;
            out->tpeak = tpeak;
            out->k = k;
        }
    }

    vfs_free(buf);
    return ok ? 0 : -1;
}

int component_engine_curve(uint32_t engine_num, ComponentEngineCurve *out)
{
    uint32_t component_id = 0;
    if (engine_component_id(engine_num, &component_id) != 0) return -1;
    return component_engine_curve_by_id(component_id, out);
}
