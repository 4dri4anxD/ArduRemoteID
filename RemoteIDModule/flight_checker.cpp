#include "flight_checker.h"
#include <Arduino.h>
#include <stdlib.h>
#include "parameters.h"

Coordinate FlightChecks::origin;

uint16_t FlightChecks::country_coords_counter;
uint16_t FlightChecks::country_coords_size;
Coordinate *FlightChecks::country_coords = nullptr;

uint16_t FlightChecks::airport_coords_counter;
uint16_t FlightChecks::airport_coords_size;
AirportCoordinate *FlightChecks::airport_coords = nullptr;

bool FlightChecks::files_read;

void FlightChecks::init()
{
    country_coords_size = 16;
    airport_coords_size = 16;
    country_coords_counter = 0;
    airport_coords_counter = 0;
    files_read = false;
    origin = {0, 0};
    if (country_coords != nullptr)
    {
        free(country_coords);
    }
    if (airport_coords != nullptr)
    {
        free(airport_coords);
    }
    country_coords = (Coordinate *)malloc(country_coords_size * sizeof(Coordinate));
    airport_coords = (AirportCoordinate *)malloc(airport_coords_size * sizeof(AirportCoordinate));
}

bool FlightChecks::check_for_near_airports()
{//Reads a file with bunch of airports and only save the ones that the drone can reach in an object array
    File full_airport_file = SPIFFS.open(FULL_AIRPORT_LIST, FILE_READ);
    if (!full_airport_file)
    {
        Serial.println("Failed to open file");
        return false;
    }

    uint32_t last_wdt_reset = millis();

    while (full_airport_file.available())
    {
        String line = full_airport_file.readStringUntil('\n');
        AirportCoordinate coord = parse_airport_coordinate(line);
        if (dc.haversine(origin.lat, origin.lon, coord.lat, coord.lon) < MAX_DRONE_DISTANCE && airport_coords_size < MAX_CLOSE_AIRPORTS_SIZE)
        {
            if (airport_coords_counter >= airport_coords_size)
            {
                if (!double_coords_array(COORDS_ARRAY_ID::AIRPORT))
                    return false;
            }

            airport_coords[airport_coords_counter] = coord;
            //Debug
            Serial.printf("Saved: %.7f,%.7f\n", airport_coords[airport_coords_counter].lat, airport_coords[airport_coords_counter].lon);
            airport_coords_counter++;
            if (!check_airports)
                check_airports = true;
        }
        reset_wdt(&last_wdt_reset);
    }
    full_airport_file.close();
    return true;
}

void FlightChecks::reset_wdt(uint32_t *last_reset)
{// Avoid wdt being triggered by setting a delay
    uint32_t now = millis();
    if (now - *last_reset > 500)
    {
        t.set_fl_status(MAV_AURELIA_CHECK_STATUS_FAIL_FLYING_NOT_ALLOWED);
        const char *init_msg = "Initializing";
        t.set_parse_fail(init_msg);
        t.update();
        delay(1);
        *last_reset = now;
    }
}

bool FlightChecks::is_flying_near_an_airport()
{//Checks if flying inside an airport area
    for (uint16_t i = 0; i < airport_coords_counter; i++)
    {
        if (dc.haversine(origin.lat, origin.lon, airport_coords[i].lat, airport_coords[i].lon) < min_distance[airport_coords[i].type])
        {
            return true;
        }
    }
    return false;
}

uint8_t FlightChecks::is_inside_polygon_file(File countries_file)
{//The same as is_inside_polygon, but reading a spiffs file instead of an object array
    bool inside = false;
    Coordinate firstCoord;
    Coordinate prevCoord;
    bool isFirstCoord = true;
    uint8_t polygon_count = 0;

    uint32_t last_wdt_reset = millis();

    while (countries_file.available())
    {
        String line = countries_file.readStringUntil('\n');
        if (line.startsWith("#"))
        {
            if (!isFirstCoord)
            {
                inside ^= checkEdge(origin.lat, origin.lon, prevCoord.lat, prevCoord.lon, firstCoord.lat, firstCoord.lon);
                if (inside)
                {
                    return polygon_count;
                }
            }
            polygon_count++;
            isFirstCoord = true;
            continue;
        }
        if (isFirstCoord)
        {
            firstCoord = parse_coordinate(line);
            prevCoord = firstCoord;
            isFirstCoord = false;
            continue;
        }
        Coordinate currentCoord = parse_coordinate(line);
        inside ^= checkEdge(origin.lat, origin.lon, prevCoord.lat, prevCoord.lon, currentCoord.lat, currentCoord.lon);
        prevCoord = currentCoord;
        reset_wdt(&last_wdt_reset);
    }
    return 0;
}

bool FlightChecks::checkEdge(double x, double y, double x1, double y1, double x2, double y2)
{//Check if an imaginary straight line hits an egde
    if (y > min(y1, y2) && y <= max(y1, y2) && x <= max(x1, x2))
    {
        if (y1 != y2)
        {
            double xinters = (y - y1) * (x2 - x1) / (y2 - y1) + x1;
            if (x1 == x2 || x <= xinters)
            {
                return true;
            }
        }
    }
    return false;
}

bool FlightChecks::check_for_near_countries()
{
    File file = SPIFFS.open(FULL_COUNTRY_LIST, FILE_READ);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return false;
    }
    //First we check if we're inside a banned country and save which country is
    is_inside_banned_country = is_inside_polygon_file(file);
    //Reset the pointer on the file reading
    file.seek(0);
    bool startedRegion = false;

    Coordinate coord1;
    Coordinate coord2;
    Coordinate prevCoord;
    Coordinate firstCoord;

    bool isFirstCoord = true;
    bool isFirstFoundCoord = false;

    uint8_t polygon_count = 0;
    uint8_t current_polygon_offset = 0;

    while (file.available())
    {
        String line = file.readStringUntil('\n');
        if (line.startsWith("#"))
        { // New polygon
            if (prevCoord.lat != firstCoord.lat && prevCoord.lon != firstCoord.lon && isFirstFoundCoord)
            { // The current Polygon isn't closed
                //Check if enough space in object
                if ((country_coords_counter + EXTRA_COORDINATES_CLOSE_POLYGON) >= country_coords_size)
                {
                    if (!double_coords_array(COORDS_ARRAY_ID::COUNTRY))
                        return false;
                }
                close_polygon(firstCoord, prevCoord, polygon_count, current_polygon_offset);
            }
            else if (isFirstFoundCoord)
            { // We have more than 1 polygon, then we separate it by a 0,0 coordinate
                if (country_coords_counter >= country_coords_size)
                {
                    if (!double_coords_array(COORDS_ARRAY_ID::COUNTRY))
                        return false;
                }
                country_coords[country_coords_counter] = {0, 0};
                country_coords_counter++;
            }
            current_polygon_offset = country_coords_counter;
            polygon_count++;
            isFirstCoord = true;
            isFirstFoundCoord = false;
            continue;
        }

        // Fisrt coord of polygon
        if (isFirstCoord)
        {
            coord1 = parse_coordinate(line);
            isFirstCoord = false;
            continue;
        }

        //Check if the current coord hits a edge (with vertices on current coord and the previous one)
        coord2 = parse_coordinate(line);
        if (distance_from_point_to_line_segment(coord1, coord2) < MAX_DRONE_DISTANCE && country_coords_size < MAX_CLOSE_BORDERS_SIZE)
        {//We only save those coordinates that the drone is able to reach
            if ((country_coords_counter + 1) >= country_coords_size)
            {//Double the object if neccesary
                if (!double_coords_array(COORDS_ARRAY_ID::COUNTRY))
                    return false;
            }
            if (coord1.lat != prevCoord.lat && coord1.lon != prevCoord.lon)
            {//None of the coords are already in the object
                country_coords[country_coords_counter] = coord1;
                country_coords_counter++;
                country_coords[country_coords_counter] = coord2;
                country_coords_counter++;
            }
            else
            {//The first coordinate is already in object, we only save the second
                country_coords[country_coords_counter] = coord2;
                country_coords_counter++;
            }

            if (!isFirstFoundCoord)
            {
                firstCoord = coord1;
                isFirstFoundCoord = true;
                check_countries = true;
            }

            prevCoord = coord2;
        }

        //Update the previous coord
        coord1 = coord2;
    }

    file.close();
    return true;
}

bool FlightChecks::double_coords_array(COORDS_ARRAY_ID coords_id)
{
    void **coord_ptr = nullptr;
    uint16_t coord_size = 0;
    size_t element_size = 0;

    //Assing pointers and size depending on the object
    switch (coords_id)
    {
    case COORDS_ARRAY_ID::COUNTRY:
        coord_ptr = (void **)&country_coords;
        //Duplicate size
        country_coords_size *= 2;
        coord_size = country_coords_size;
        element_size = sizeof(Coordinate);
        break;

    case COORDS_ARRAY_ID::AIRPORT:
        coord_ptr = (void **)&airport_coords;
        //Duplicate size
        airport_coords_size *= 2;
        coord_size = airport_coords_size;
        element_size = sizeof(AirportCoordinate);
        break;

    case COORDS_ARRAY_ID::PRISON:
        return false;
        break;

    default:
        return false;
    }

    void *temp_ptr = realloc(*coord_ptr, (coord_size)*element_size);
    if (temp_ptr == NULL)
    {
        Serial.println("Realloc failed");
        free(*coord_ptr);
        return false;
    }

    //We just double size of the current object
    *coord_ptr = temp_ptr;
    return true;
}

bool FlightChecks::is_inside_polygon(uint8_t offset)
{
    if (country_coords_counter < 3)
    {//It's not a polygon
        return false;
    }

    bool inside_generated_polygon = false;
    int count = 0;
    int next;
    Coordinate firstCoordinate = {0, 0};

    for (int i = offset; i < country_coords_counter - 1; i++)
    {
        if (country_coords[i].lat == 0 && country_coords[i].lon == 0)
        {//We reached the beginning of the next polygon
            firstCoordinate = {0, 0};
            continue;
        }
        next = (i + 1);
        if (country_coords[next].lat == 0 && country_coords[next].lon == 0)
        {//We reached the end of the current polygon
            if (checkEdge(origin.lat, origin.lon, country_coords[i].lat, country_coords[i].lon, firstCoordinate.lat, firstCoordinate.lon))
            {
                count++;
            }
        }
        else
        {//Process current polygon
            if (firstCoordinate.lat == 0 && firstCoordinate.lon == 0)
            {//Saves the first coordinate
                firstCoordinate.lat = country_coords[i].lat;
                firstCoordinate.lon = country_coords[i].lon;
            }

            if (checkEdge(origin.lat, origin.lon, country_coords[i].lat, country_coords[i].lon, country_coords[next].lat, country_coords[next].lon))
            {//Checks if the current location hits an edge
                count++;
            }
        }
        // Serial.printf("Coordenada country %d: lat = %.7f, lon = %.7f\n", i + 1, country_coords[i].lat, country_coords[i].lon);
    }
    if (count % 2 == 1)
    {//ray-casting algorithm, if the number is odd, then we are inside a polygon
        inside_generated_polygon = true;
    }
    return inside_generated_polygon;
}

void FlightChecks::close_polygon(Coordinate firstCoord, Coordinate lastCoord, uint8_t polygon_count, uint8_t offset)
{
    double bearing_first = calculate_bearing(origin.lat, origin.lon, firstCoord.lat, firstCoord.lon);
    double bearing_last = calculate_bearing(origin.lat, origin.lon, lastCoord.lat, lastCoord.lon);

    Coordinate new_first_coord = destination_point(firstCoord.lat, firstCoord.lon, LINE_LENGHT, bearing_first);
    Coordinate new_last_coord = destination_point(lastCoord.lat, lastCoord.lon, LINE_LENGHT, bearing_last);

    double midpoint_lat = (new_first_coord.lat + new_last_coord.lat) / 2;
    double midpoint_lon = (new_first_coord.lon + new_last_coord.lon) / 2;

    double bearing_origin_midpoint = calculate_bearing(origin.lat, origin.lon, midpoint_lat, midpoint_lon);
    Coordinate close_polygon_a = destination_point(origin.lat, origin.lon, LINE_LENGHT, bearing_origin_midpoint);
    country_coords[country_coords_counter] = new_last_coord;
    country_coords_counter++;
    country_coords[country_coords_counter] = close_polygon_a;
    country_coords_counter++;
    country_coords[country_coords_counter] = new_first_coord;
    country_coords_counter++;
    country_coords[country_coords_counter] = firstCoord;
    country_coords_counter++;
    country_coords[country_coords_counter] = {0, 0};
    country_coords_counter++;
    check_final_polygon(bearing_origin_midpoint, polygon_count, offset);
}

void FlightChecks::check_final_polygon(double bearing_origin_midpoint, uint8_t polygon_count, uint8_t offset)
{   //Checks if the current coordinate is inside or outside the final polygon depending on wether it is on a restricted area
    bool inside_generated_polygon = is_inside_polygon(offset);//check if is inside the polygon
    if ((inside_generated_polygon && is_inside_banned_country != polygon_count) || (!inside_generated_polygon && is_inside_banned_country == polygon_count))
    {//Changes a key coordinate to put in or take out the drone coordinate to the polygon
        Coordinate close_polygon_b = destination_point(origin.lat, origin.lon, (LINE_LENGHT * -1), bearing_origin_midpoint);
        country_coords[country_coords_counter - EXTRA_COORDINATES_CLOSE_POLYGON] = close_polygon_b;
    }
}

float FlightChecks::distance_from_point_to_line_segment(Coordinate coord1, Coordinate coord2)
{
    double py = origin.lon;
    double px = origin.lat;
    double y1 = coord1.lon;
    double x1 = coord1.lat;
    double y2 = coord2.lon;
    double x2 = coord2.lat;

    // Calcute segment length
    double seg_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    if (seg_len_sq < 1e-9)
    {
        return dc.haversine(px, py, x1, y1); // It's not a segment but a point
    }

    // Projection of the point on the segment
    double t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / seg_len_sq;
    t = (t < 0) ? 0 : (t > 1) ? 1
                              : t;

    // Calculate the point on the nearest segment
    double nearest_x = x1 + t * (x2 - x1);
    double nearest_y = y1 + t * (y2 - y1);

    return dc.haversine(px, py, nearest_x, nearest_y);
}

Coordinate FlightChecks::parse_coordinate(String line)
{
    Coordinate coord;
    int commaIndex = line.indexOf(',');
    if (commaIndex != -1)
    {
        String latStr = line.substring(0, commaIndex);
        String lonStr = line.substring(commaIndex + 1);
        coord.lat = latStr.toDouble();
        coord.lon = lonStr.toDouble();
    }
    return coord;
}

AirportCoordinate FlightChecks::parse_airport_coordinate(String line)
{
    AirportCoordinate coord;
    int firstComma = line.indexOf(',');
    int secondComma = line.indexOf(',', firstComma + 1);
    if (firstComma != -1 && secondComma != -1)
    {
        coord.type = line.substring(0, firstComma).toInt();
        coord.lat = line.substring(firstComma + 1, secondComma).toDouble();
        coord.lon = line.substring(secondComma + 1).toDouble();
    }
    return coord;
}

double FlightChecks::degrees_to_radians(double degrees)
{
    return degrees * PI / 180.0;
}

double FlightChecks::radians_to_degrees(double radians)
{
    return radians * 180.0 / PI;
}

// Function for calculating the bearing angle between two points
double FlightChecks::calculate_bearing(double lat1, double lon1, double lat2, double lon2)
{
    lat1 = degrees_to_radians(lat1);
    lon1 = degrees_to_radians(lon1);
    lat2 = degrees_to_radians(lat2);
    lon2 = degrees_to_radians(lon2);

    double dlon = lon2 - lon1;

    double x = sin(dlon) * cos(lat2);
    double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);

    // Calculate the initial heading angle
    double initial_bearing = atan2(x, y);

    initial_bearing = initial_bearing * 180.0 / PI;

    double compass_bearing = fmod((initial_bearing + 360.0), 360.0);

    return compass_bearing;
}

Coordinate FlightChecks::destination_point(double lat, double lon, double distance, double bearing)
{
    double lat1 = degrees_to_radians(lat);
    double lon1 = degrees_to_radians(lon);
    double bearing_rad = degrees_to_radians(bearing);

    // Calculate latitude of destination point
    double lat2 = asin(sin(lat1) * cos(distance / EARTH_RADIUS) +
                       cos(lat1) * sin(distance / EARTH_RADIUS) * cos(bearing_rad));

    // Calculate longitude of destination point
    double lon2 = lon1 + atan2(sin(bearing_rad) * sin(distance / EARTH_RADIUS) * cos(lat1),
                               cos(distance / EARTH_RADIUS) - sin(lat1) * sin(lat2));

    double lat_ret = round(radians_to_degrees(lat2) * 1e7) / 1e7;
    double lon_ret = round(radians_to_degrees(lon2) * 1e7) / 1e7;
    Coordinate ret = {lat_ret, lon_ret};
    return ret;
}

flying_banned FlightChecks::is_flying_allowed()
{
    flying_banned fb;
    fb.allowed = FLIGHT_BANNED_REASON::NO_BAN;
    strncpy(fb.reason, "", sizeof(fb.reason) - 1);

    if ((g.options & OPTIONS_BYPASS_AIRPORT_CHECKS) && (g.options & OPTIONS_BYPASS_COUNTRY_CHECKS))
    {//If by passed
        return fb;
    }

    if (origin.lat == 0 && origin.lon == 0)
    {//If we have no GPS position, then return
        fb.allowed = FLIGHT_BANNED_REASON::GPS;
        strncpy(fb.reason, "GPS", sizeof(fb.reason) - 1);
        return fb;
    }

    if (!files_read)
    {//Only enters the first time when powered up
        if (t.get_ack_request_status() == MAV_AURELIA_UTIL_ACK_REQUEST_DONE)
        {
            bool passed = true;
            if (!(g.options & OPTIONS_BYPASS_AIRPORT_CHECKS))
            {
                passed &= check_for_near_airports();
            }
            if (!(g.options & OPTIONS_BYPASS_COUNTRY_CHECKS))
            {
                passed &= check_for_near_countries();
            }
            if (passed)
            {
                //Debug
                for (int i = 0; i < country_coords_counter; i++)
                {
                    Serial.printf("Save country coordinate %d: lat = %.7f, lon = %.7f\n", i + 1, country_coords[i].lat, country_coords[i].lon);
                }
                files_read = true;
            }
            else
            {
                free(country_coords);
                free(airport_coords);
                fb.allowed = FLIGHT_BANNED_REASON::FILE_ERROR;
                strncpy(fb.reason, "FILE", sizeof(fb.reason) - 1);
                return fb;
            }
        }
    }

    if (!(g.options & OPTIONS_BYPASS_AIRPORT_CHECKS) && (check_airports ? is_flying_near_an_airport() : false))
    {
        fb.allowed = FLIGHT_BANNED_REASON::AIRPORT;
        strncpy(fb.reason, "AIRPORT", sizeof(fb.reason) - 1);
        return fb;
    }

    if (!(g.options & OPTIONS_BYPASS_COUNTRY_CHECKS) && (check_countries ? is_inside_polygon() : is_inside_banned_country > 0 ? true
                                                                                                                              : false))
    {
        fb.allowed = FLIGHT_BANNED_REASON::COUNTRY;
        strncpy(fb.reason, "COUNTRY", sizeof(fb.reason) - 1);
        return fb;
    }

    return fb;
}