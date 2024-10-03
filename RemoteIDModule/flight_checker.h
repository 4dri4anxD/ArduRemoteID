#pragma once

#include <FS.h>
#include "SPIFFS.h"
#include "distance_checker.h"
#include <math.h>
#include "spiffs_utils.h"
#include "transport.h"

#define FULL_AIRPORT_LIST "/world_airport_list.txt"
#define FULL_COUNTRY_LIST "/banned_countries.txt"

#define LINE_LENGHT 200 //Distance (in Km) to which a new point will be projected to close the polygon
#define MAX_DRONE_DISTANCE 55 // Maximum distance (in km) that the drone can travel before running out of battery
#define EXTRA_COORDINATES_CLOSE_POLYGON 4 //Elements in the array necessary to ensure that the object will not run out of memory to close the polygon

#define MAX_CLOSE_AIRPORTS_SIZE 1024 //Maximum quantity of elements in airport coord object
#define MAX_CLOSE_BORDERS_SIZE 1024 //Maximum quantity of elements in country coord object

/*
Airports types
0-Large airport
1-Medium airport
2-Small airport
3-Heliport
4-Seaplane base
5-Hot-air balloons base
*/
// const float min_distance[6] = {4, 0.035, 1, 0.2, 1, 0.5}; // Minimum distance (in km) that the drone must be from each type of airport to be able to fly
const float min_distance[6] = {0.035, 0.035, 0.035, 0.035, 0.035, 0.035}; // Minimum distance (in km) that the drone must be from each type of airport to be able to fly

enum class COORDS_ARRAY_ID : uint8_t
{//For resizing objects
    AIRPORT = 0,
    COUNTRY = 1,
    PRISON = 2,
};

typedef struct
{//For countries
    double lat;
    double lon;
} Coordinate;

typedef struct
{//For airports
    uint8_t type;
    double lat;
    double lon;
} AirportCoordinate;

class FlightChecks {
public:
    FlightChecks(Transport &transport) : t(transport) {};

    flying_banned is_flying_allowed();
    void update_location(double lat, double lon)
    {
        origin.lat = lat;
        origin.lon = lon;
    }

    bool get_files_read()
    {
        return files_read;
    }

    void init();

private:
    bool check_for_near_airports();
    bool check_for_near_countries();

    bool is_flying_near_an_airport();
    uint8_t is_inside_polygon_file(File near_countries_file);
    bool is_inside_polygon(uint8_t offset = 0);

    bool checkEdge(double x, double y, double x1, double y1, double x2, double y2);
    float distance_from_point_to_line_segment(Coordinate coord1, Coordinate coord2);
    double calculate_bearing(double lat1, double lon1, double lat2, double lon2);
    Coordinate destination_point(double lat, double lon, double distance, double bearing);

    double degrees_to_radians(double degrees);
    double radians_to_degrees(double radians);

    bool double_coords_array(COORDS_ARRAY_ID coords_id);

    void close_polygon(Coordinate firstCoord, Coordinate lastCoord, uint8_t polygon_count, uint8_t offset);
    void check_final_polygon(double bearing_origin_midpoint, uint8_t polygon_count, uint8_t offset);

    void reset_wdt(uint32_t *last_reset);

    Coordinate parse_coordinate(String str);
    AirportCoordinate parse_airport_coordinate(String line);

    static bool files_read;
    bool check_airports = false;
    bool check_countries = false;
    uint8_t is_inside_banned_country = 0;

    static Coordinate origin;

    static uint16_t country_coords_counter;
    static uint16_t country_coords_size;
    static Coordinate *country_coords;

    static uint16_t airport_coords_counter;
    static uint16_t airport_coords_size;
    static AirportCoordinate *airport_coords;

    DistanceCheck dc;
    Transport &t;
};