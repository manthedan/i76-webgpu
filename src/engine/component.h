#ifndef COMPONENT_H
#define COMPONENT_H

#include <stdint.h>

/* Authored engine curve selected by a VCF/engsnd.dat ENG NUM. `component_id`
 * is engsnd.dat's zero-based ENG COMP ID (eng01 = 0). Strings and brake/
 * suspension fields are intentionally outside this physics consumer.
 * Returns 0 on success; -1 for absent/malformed data or an unmapped id. */
typedef struct {
    uint32_t component_id;
    float tpeak;
    float k;
} ComponentEngineCurve;

int component_engine_curve(uint32_t engine_num, ComponentEngineCurve *out);

/* Evidence-backed installed-component override seam. Normal P01's campaign
 * car was observed with eng02 even though its garage VCF carries ENG NUM 3.
 * The id remains zero-based; ordinary VCF consumption uses the function above. */
int component_engine_curve_by_id(uint32_t component_id,
                                 ComponentEngineCurve *out);

#endif
