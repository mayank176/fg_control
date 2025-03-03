#ifndef UDP_IO_H
#define UDP_IO_H

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/array.hpp>

//https://wiki.flightgear.org/Aircraft_properties_reference

struct FlightState {
    double roll_deg {};      // [degrees]
    double pitch_deg {};     // [degrees]
    double heading_deg {};   // [degrees]
    double altitude_ft {};   // [feet]
    double airspeed_kt {};   // [knots]
    double roll_rate {};     // [degree per second]
    double pitch_rate {};    // [degree per second]
    double yaw_rate {};      // [degree per second]
    double vertical_speed_fps {};
    double latitude {};    // in degrees
    double longitude {};   // in degrees
};

struct FlightControls {
    double aileron {};    // [-1.0, 1.0]
    double elevator {};   // [-1.0, 1.0]
    double rudder {};     // [-1.0, 1.0]
    double throttle {};   // [-1.0, 1.0]
};

struct Waypoint {
    double latitude;   // in degrees
    double longitude;   // in degrees
    double altitude;    // in feet
    double speed;       // in knots
};

class UDP_IO {
public:
    const std::string receive_host;
    const int receive_port;

    const std::string send_host;
    const int send_port;

    bool is_initialized;

    UDP_IO(const std::string &receive_host, int receive_port,
        const std::string &send_host, int send_port);
    
    void initialize();
    
    bool receiveFlightState(FlightState &state);
    bool sendFlightControls(const FlightControls &controls);
    float swap_float_bytes(float value);

    // Boost.ASIO members
    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket receive_socket;
    boost::asio::ip::udp::socket send_socket;
    boost::asio::ip::udp::endpoint send_endpoint;

};

#endif //UDP_IO_H 