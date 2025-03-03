#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "udp_io.h"
#include <memory>
#include <string>

class AircraftController {
public:
    AircraftController();
    ~AircraftController() = default;
    
    void initialize();
    void setFlightPath(const std::vector<Waypoint>& path);
    void updatePosition(double lat, double lon);
    void trackLineSegment();
    
    void run();
    
    void setTargetAltitude(double altitude);
    void setTargetHeading(double heading);
    void setTargetSpeed(double speed);
    
    double getTargetAltitude() const;
    double getTargetHeading() const;
    double getTargetSpeed() const;
    
    void printStatus();

    // Last received state
    FlightState current_state;
    
    void stop() { should_run = false; }
    
private:
    std::vector<Waypoint> flightPath;
    size_t currentWaypointIndex = 0;
    double currentLatitude;
    double currentLongitude;
    
    double calculateBearing(double lat1, double lon1, double lat2, double lon2);
    double calculateCrossTrackError(double currentLat, double currentLon, Waypoint start, Waypoint end);
    double calculateDistance(double lat1, double lon1, double lat2, double lon2);
    
    double calculateElevatorCommand(const FlightState& state);
    double calculateRudderCommand(const FlightState& state);
    double calculateAileronCommand(const FlightState& state);
    double calculateThrustCommand(const FlightState& state);

    std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
    // UDP communication interface
    std::unique_ptr<UDP_IO> udp_interface;

    double integral;    // Accumulated error
    double integral_e;
    double last_airspeed;  // Previous error
    
    double target_altitude;
    double target_heading;
    double target_speed;

    bool should_run;
    
    // Execute a single control update cycle
    bool updateControl();
};

#endif