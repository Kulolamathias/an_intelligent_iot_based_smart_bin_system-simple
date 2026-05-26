#ifndef GEO_UTILS_H
#define GEO_UTILS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculate Haversine distance between two points.
 * @param lat1 Latitude of first point (degrees)
 * @param lon1 Longitude of first point (degrees)
 * @param lat2 Latitude of second point (degrees)
 * @param lon2 Longitude of second point (degrees)
 * @return Distance in meters (rounded to nearest meter)
 */
uint32_t geo_distance(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief Calculate initial bearing from point1 to point2.
 * @param lat1 Latitude of start (degrees)
 * @param lon1 Longitude of start (degrees)
 * @param lat2 Latitude of end (degrees)
 * @param lon2 Longitude of end (degrees)
 * @return Bearing in degrees (0‑360, 0 = North)
 */
double geo_bearing(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief Convert bearing to cardinal / intercardinal string.
 * @param bearing_deg Bearing in degrees
 * @return One of: "N", "NE", "E", "SE", "S", "SW", "W", "NW"
 */
const char* geo_bearing_to_cardinal(double bearing_deg);

#ifdef __cplusplus
}
#endif

#endif