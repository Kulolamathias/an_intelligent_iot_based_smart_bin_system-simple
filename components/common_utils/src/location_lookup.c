#include "location_lookup.h"
#include <string.h>
#include <math.h>

/* -------------------------------------------------------------------------
 * Hardcoded known places from GPS logs (UDOM campus)
 * ------------------------------------------------------------------------- */
typedef struct {
    double lat;
    double lon;
    const char *name;
} known_place_t;

static const known_place_t s_known_places[] = {
    // Block / outer block area
    { -6.215477, 35.813713, "Block 37A" },
    { -6.215380, 35.813753, "Block 37A" },
    { -6.212507, 35.814339, "NJE Block 1" },
    { -6.212480, 35.814355, "NJE Block 1" },
    // CIVE area (initial fix, repeated log entries)
    { -6.215477, 35.813713, "CIVE Building" },
    { -6.215476, 35.813713, "CIVE Building" },
    { -6.215429, 35.813725, "CIVE Building" },
    // Ecowater / library side points
    { -6.216023, 35.812976, "Ecowater" },
    { -6.216161, 35.813009, "Ecowater" },
    // Library area
    { -6.216035, 35.812565, "Library" },
    { -6.216025, 35.812537, "Library" },
    { -6.216042, 35.812528, "Library" },
    { -6.216006, 35.812602, "Library" },
    // Auditorium
    { -6.216501, 35.811178, "Auditorium" },
    { -6.216660, 35.811174, "Auditorium" },
    // Administration
    { -6.217123, 35.809566, "Administration" },
    { -6.217114, 35.809432, "Administration" },
    // Academic / EGA / LRA side
    { -6.216474, 35.808748, "Academic" },
    { -6.216157, 35.808720, "Academic" },
    { -6.215343, 35.808053, "EGA" },
    { -6.215155, 35.808057, "EGA" },
    { -6.214263, 35.808354, "LRA" },
    { -6.214129, 35.808398, "LRA" },
    // Cafeteria
    { -6.215188, 35.811422, "Cafeteria" },
    { -6.215047, 35.811393, "Cafeteria" },
    // LRB (Law Reports Building or similar)
    { -6.214217, 35.809001, "LRB" },
    { -6.214191, 35.808956, "LRB" },
    { -6.214165, 35.809135, "LRB" },
    // LRB parking, from assessment GPS logs
    { -6.213407, 35.810034, "LRB Parking" },
    { -6.213631, 35.809574, "LRB Parking" },
    { -6.213529, 35.809558, "LRB Parking" },
    { -6.213425, 35.809603, "LRB Parking" },
    // Multipurpose Lab
    { -6.214766, 35.809391, "MULTLAB" },
    { -6.214862, 35.809403, "MULTLAB" },
    { -6.215041, 35.809420, "MULTLAB" },
    // Sports Field
    { -6.217500, 35.806702, "Sports Field" },
    // Additional points from logs (near library/auditorium)
    { -6.216023, 35.812976, "Library (South)" },
    { -6.216161, 35.813009, "Library (East)" },
    { -6.216186, 35.813021, "Library (East)" },
    { -6.216829, 35.810876, "Auditorium (East)" },
    { -6.216866, 35.810840, "Auditorium (East)" },
    // Cafeteria surrounding
    { -6.215177, 35.811414, "Cafeteria (North)" },
    { -6.215009, 35.811495, "Cafeteria (East)" },
    // End marker
};

#define NUM_KNOWN_PLACES (sizeof(s_known_places) / sizeof(s_known_places[0]))

/* Maximum distance (meters) to consider a campus-area name match.
 * GPS readings around buildings can drift, so keep this practical for demos.
 */
#define MATCH_DISTANCE_M 120.0

/* Local Haversine function (avoid dependency on geo_utils for now) */
static double haversine(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0;
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double dphi = (lat2 - lat1) * M_PI / 180.0;
    double dlambda = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dphi / 2.0) * sin(dphi / 2.0) +
               cos(phi1) * cos(phi2) *
               sin(dlambda / 2.0) * sin(dlambda / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return R * c;
}

bool location_lookup_find(double lat, double lon, char *name_buf, size_t buf_size)
{
    if (name_buf == NULL || buf_size == 0) {
        return false;
    }

    double best_dist = MATCH_DISTANCE_M + 1.0;
    const char *best_name = NULL;

    for (size_t i = 0; i < NUM_KNOWN_PLACES; i++) {
        double dist = haversine(lat, lon, s_known_places[i].lat, s_known_places[i].lon);
        if (dist < best_dist) {
            best_dist = dist;
            best_name = s_known_places[i].name;
        }
    }

    if (best_name != NULL && best_dist <= MATCH_DISTANCE_M) {
        strncpy(name_buf, best_name, buf_size - 1);
        name_buf[buf_size - 1] = '\0';
        return true;
    }

    name_buf[0] = '\0';
    return false;
}
