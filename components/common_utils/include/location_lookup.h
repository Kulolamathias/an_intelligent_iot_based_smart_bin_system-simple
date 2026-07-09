#ifndef LOCATION_LOOKUP_H
#define LOCATION_LOOKUP_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Find a human‑readable place name for given coordinates.
 * @param lat Latitude (degrees)
 * @param lon Longitude (degrees)
 * @param name_buf Output buffer (will be null‑terminated)
 * @param buf_size Size of name_buf (recommended at least 32)
 * @return true if a name was found, false otherwise
 */
bool location_lookup_find(double lat, double lon, char *name_buf, size_t buf_size);

/**
 * @brief Find a canonical assessment zone using a center/radius model.
 * @param lat Latitude (degrees)
 * @param lon Longitude (degrees)
 * @param name_buf Output canonical zone name
 * @param buf_size Size of name_buf
 * @param distance_m Optional distance from zone center
 * @param radius_m Optional matching radius used for the zone
 * @return true if coordinates fall inside a known zone
 */
bool location_lookup_find_zone(double lat,
                               double lon,
                               char *name_buf,
                               size_t buf_size,
                               double *distance_m,
                               double *radius_m);

/**
 * @brief (Future) Add or update a place in NVS – stub for now.
 */
// esp_err_t location_lookup_add(double lat, double lon, const char *name);

#ifdef __cplusplus
}
#endif

#endif
