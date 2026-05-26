#include "geo_utils.h"
#include <math.h>

#define EARTH_RADIUS_M 6371000.0
#define DEG_TO_RAD (M_PI / 180.0)

uint32_t geo_distance(double lat1, double lon1, double lat2, double lon2)
{
    double phi1 = lat1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double dphi = (lat2 - lat1) * DEG_TO_RAD;
    double dlambda = (lon2 - lon1) * DEG_TO_RAD;

    double a = sin(dphi / 2.0) * sin(dphi / 2.0) +
               cos(phi1) * cos(phi2) *
               sin(dlambda / 2.0) * sin(dlambda / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    double distance = EARTH_RADIUS_M * c;
    return (uint32_t)(distance + 0.5);
}

double geo_bearing(double lat1, double lon1, double lat2, double lon2)
{
    double phi1 = lat1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double dlambda = (lon2 - lon1) * DEG_TO_RAD;

    double y = sin(dlambda) * cos(phi2);
    double x = cos(phi1) * sin(phi2) -
               sin(phi1) * cos(phi2) * cos(dlambda);
    double bearing = atan2(y, x) / DEG_TO_RAD;
    if (bearing < 0) bearing += 360.0;
    return bearing;
}

const char* geo_bearing_to_cardinal(double bearing_deg)
{
    if (bearing_deg < 22.5) return "N";
    if (bearing_deg < 67.5) return "NE";
    if (bearing_deg < 112.5) return "E";
    if (bearing_deg < 157.5) return "SE";
    if (bearing_deg < 202.5) return "S";
    if (bearing_deg < 247.5) return "SW";
    if (bearing_deg < 292.5) return "W";
    if (bearing_deg < 337.5) return "NW";
    return "N";
}