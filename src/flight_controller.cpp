#include "flight_controller.h"

void AircraftController::initialize() {    
    // Create UDP interface
    udp_interface = std::make_unique<UDP_IO>("127.0.0.1", 5505, "127.0.0.1", 5506);
    
    try {
        udp_interface->initialize();
        std::cout << "Controller initialized and ready." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing UDP interface: " << e.what() << std::endl;
        throw;
    }
    
}

double AircraftController::calculateElevatorCommand(const FlightState& state) {
    
    double Kr = 0.005;       // Increase pitch rate damping
    double K_theta = -0.03;  // Increase pitch attitude control
    double Kh = 0.05;       // Slightly increase altitude gain
    
    //Outer Loop
    double altitude_error = target_altitude - state.altitude_ft;
    double theta_r = Kh*altitude_error;
    theta_r = std::max(-15.0, std::min(theta_r, 20.0));

    //Middle Loop
    double pitch_angle_error = (theta_r - state.pitch_deg)*K_theta;
    pitch_angle_error = std::max(-0.3, std::min(pitch_angle_error, 0.3));

    //Inner Loop
    double elevator_command = pitch_angle_error - state.pitch_rate*Kr;

    // std::cout << "altitude_error: " << altitude_error << std::endl;
    // std::cout << "Kh*altitude_error: " << theta_r << std::endl;
    // std::cout << "pitch_angle_error_k_theta : " << pitch_angle_error << std::endl;
    // std::cout << "elevator_cmd: " << elevator_command << std::endl;

    // Limit elevator command to valid range [-1, 1]
    return std::max(-1.0, std::min(elevator_command, 1.0));
}

double AircraftController::calculateAileronCommand(const FlightState& state) {
    
    double Kp = 0.004; // Roll rate damper (inner loop)
    double Kphi = 0.065; //middle loop (roll angle error)
    double Kpsi = 0.4; //outer loop (heading error)
    
    //Outer Loop
    double heading_error = target_heading - state.heading_deg;
    if (heading_error > 180) heading_error -= 360;
    if (heading_error < -180) heading_error += 360;
    double target_roll_deg = Kpsi * heading_error;
    target_roll_deg = std::max(-25.0, std::min(target_roll_deg, 25.0));
    
    //Middle Loop
    double roll_error = target_roll_deg - state.roll_deg;
    double roll_rate_command = Kphi * roll_error;
    
    //Inner loop ---- Limit commanded roll rate
    roll_rate_command = std::max(-15.0, std::min(roll_rate_command, 15.0));
    double aileron_cmd = roll_rate_command - Kp*state.roll_rate;
    
    aileron_cmd = std::max(-1.0, std::min(aileron_cmd, 1.0));

    // std::cout << "heading_error: " << heading_error << std::endl;
    // std::cout << "target_roll_deg: " <<target_roll_deg  <<std::endl;
    // std::cout << "roll error*Kphi: " << roll_rate_command <<std::endl;
    // std::cout << "aileron_cmd" << aileron_cmd << std::endl;

    return aileron_cmd;
}

double AircraftController::calculateRudderCommand(const FlightState& state) {
    // Yaw rate damper (inner loop)
    double Kr = 0.02;
    double rudder_cmd = -Kr * state.yaw_rate; 
    rudder_cmd = std::max(-1.0, std::min(rudder_cmd, 1.0));
    return rudder_cmd;
}

double AircraftController::calculateThrustCommand(const FlightState& state) {
    // Calculate energy errors
    double altitude_error = target_altitude - state.altitude_ft;
    double kinetic_energy_error = (target_speed * target_speed - 
                                  state.airspeed_kt * state.airspeed_kt);
    
    // Energy controller gains
    const double Kpe = 0.03;   // Potential energy gain
    const double Kke = 0.006;   // Kinetic energy gain
    const double Kte = 0.15;    // Total energy to throttle gain
    const double Kpitch = 0.1;  // Energy distribution to pitch gain
    
    //trim values:
    const double k_power_trim = 0.00007; 
    const double k_pitch_trim = 200.0;    // not a gain, but a trimming coefficient

    // Total energy error (combination of potential and kinetic energy)
    double total_energy_error = Kpe * altitude_error + Kke * kinetic_energy_error;
    
    // Energy distribution error (difference between potential and kinetic)
    double energy_distribution_error = Kpe * altitude_error - Kke * kinetic_energy_error;
    
    // Use throttle to control total energy
    double power_trim = k_power_trim * state.airspeed_kt*state.airspeed_kt;
    double throttle = power_trim + Kte * total_energy_error;
    
    // Use pitch to control energy distribution
    double pitch_trim = k_pitch_trim / state.airspeed_kt;
    double target_pitch = pitch_trim + Kpitch * energy_distribution_error;
    
    // Apply limits
    throttle = std::max(-1.0, std::min(throttle, 1.0));
    // std::cout << "throttle: " << throttle << std::endl;
    target_pitch = std::max(-10.0, std::min(target_pitch, 15.0)); //currently designed to use only throttle
    
    return -throttle; //minus sign for flightgear
}

//---------------TECS controller from: https://ntrs.nasa.gov/api/citations/19870017485/downloads/19870017485.pdf ---------------
// double AircraftController::calculateThrustCommand(const FlightState& state) {

//     auto current_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> time_diff = current_time - last_time;
//     double dt = time_diff.count();
//     if (dt < 0.001) dt = 0.001;
//     // Constants
//     const double g = 32.174;  // ft/s^2 (acceleration due to gravity)
    
//     // gamma = theta - alpha, which is approximated as:
//     double tas_fps = state.airspeed_kt * 1.68781; // Convert knots to feet per second
//     double gamma = 0.0;
//     if (tas_fps > 10.0) { // Avoid division by zero
//         gamma = std::asin(state.vertical_speed_fps / tas_fps) * 57.2958; // Convert to degrees
//     }
//     double Kspeed = 1.5;
//     double airspeed_acceleration = (target_speed - last_airspeed)*Kspeed; 
//     double V_dot_over_g = airspeed_acceleration*1.68781 / g;
    
//     const double K_alt = 0.02;  // Gain for altitude error to gamma conversion
//     double altitude_error = target_altitude - state.altitude_ft;
//     double target_gamma = K_alt * altitude_error;
//     std::cout << "target_gamme: " << target_gamma <<std::endl;
//     target_gamma = std::max(-0.1, std::min(target_gamma, 0.1));
    
//     double gamma_error = target_gamma - gamma;
//     double V_dot_error = ((target_speed - state.airspeed_kt)*Kspeed)/g  - V_dot_over_g;
    
//     // Calculate specific energy rate error divided by velocity (equation 6)
//     double specific_energy_rate_error_over_V = gamma_error + V_dot_error;
    
//     // PI controller gains for throttle (from equation 5)
//     const double Ktp = 0.5;   // Proportional gain
//     const double Kti = 0.1;   // Integral gain
    
//     // Update integral term with anti-windup protection
//     integral += specific_energy_rate_error_over_V * dt;
//     std::cout << "integral: " << integral <<std::endl;
//     integral = std::max(-8.0, std::min(integral, 8.0));
    
//     double throttle_command = Ktp*specific_energy_rate_error_over_V + Kti*integral;
//     // Limit throttle command to valid range [0, 1]
//     throttle_command = std::max(0.0, std::min(throttle_command, 0.99));
//     // std::cout << "Throttle_command: " << -throttle_command << std::endl;

//     std::cout << "Target Speed: " << target_speed << " kts, Current: " << state.airspeed_kt << " kts" << std::endl;
//     std::cout << "Airspeed Error: " << target_speed - state.airspeed_kt << " kts" << std::endl;
//     std::cout << "Airspeed Accel: " << airspeed_acceleration << " kts/s" << std::endl;
//     std::cout << "V_dot_error: " << V_dot_error << std::endl;
//     std::cout << "Gamma Error: " << gamma_error << std::endl;
//     std::cout << "Energy Rate Error: " << specific_energy_rate_error_over_V << std::endl;
//     std::cout << "Throttle Command: " << throttle_command << std::endl;

//     double Kep = 0.5;
//     double Kei = 0.1;

//     double energy_distribution_error = gamma_error - V_dot_error;

//     integral_e += energy_distribution_error * dt;
//     integral_e = std::max(-5.0, std::min(integral, 5.0));

//     double elevator_command = (Kep*energy_distribution_error) + Kei*integral_e;
//     elevator_command = std::max(-1.0, std::min(elevator_command, 1.0));
    
//     last_airspeed = state.airspeed_kt;
//     last_time = current_time;
    
//     return -throttle_command;
// }


bool AircraftController::updateControl() {
    if (!udp_interface->receiveFlightState(current_state)) {
        return false;
    }
    
    double elevator = calculateElevatorCommand(current_state);
    double ailerons = calculateAileronCommand(current_state);
    double rudder = calculateRudderCommand(current_state);
    double throttle = calculateThrustCommand(current_state);

    FlightControls commands = {
        .aileron = static_cast<float>(ailerons),
        .elevator = static_cast<float>(elevator),
        .rudder = static_cast<float>(rudder),
        .throttle = static_cast<float>(throttle)
    };
    
    udp_interface->sendFlightControls(commands);
    return true;
}

void AircraftController::printStatus() {
    std::cout << "======= Aircraft Controller Status =======" << std::endl;
    // std::cout << "Target values:" << std::endl;
    // std::cout << "  Altitude: " << target_altitude << " ft" << std::endl;
    // std::cout << "  Heading: " << target_heading << " deg" << std::endl;
    // std::cout << "  Speed: " << target_speed << " knots" << std::endl;
    
    // std::cout << "Current state:" << std::endl;
    // std::cout << "  Altitude: " << current_state.altitude_ft << " ft" << std::endl;
    // std::cout << "  Altitude error: " << target_altitude - current_state.altitude_ft << " ft" << std::endl;
    // std::cout << "  Vertical speed: " << current_state.vertical_speed_fps * 60.0 << " ft/min" << std::endl;
    // std::cout << "  Pitch: " << current_state.pitch_deg << " deg" << std::endl;
    // std::cout << "  Pitch rate: " << current_state.pitch_rate << " deg/s" << std::endl;
    // std::cout << "==========================================" << std::endl;
}

void AircraftController::setTargetAltitude(double altitude) {
    target_altitude = altitude;
}

void AircraftController::setTargetHeading(double heading) {
    target_heading = heading;
}

void AircraftController::setTargetSpeed(double speed){
    target_speed = speed;

}

void AircraftController::run() {
    std::cout << "Controller running. Press Ctrl+C to stop." << std::endl;
    // std::cout << "Target values:" << std::endl;
    // std::cout << "  Altitude: " << target_altitude << " ft" << std::endl;
    // std::cout << "  Heading: " << target_heading << " deg" << std::endl;
    // std::cout << "  Speed: " << target_speed << " knots" << std::endl;
    
    should_run = true;

    const std::chrono::milliseconds waypoint_update_rate(500);
    auto last_waypoint_update = std::chrono::steady_clock::now();

    while (should_run) {
        auto now = std::chrono::steady_clock::now();
        bool updated = updateControl();

        if (now - last_waypoint_update >= waypoint_update_rate) {
            double currentLat = current_state.latitude;
            double currentLon = current_state.longitude;
            updatePosition(currentLat, currentLon);
            trackLineSegment();
            last_waypoint_update = now;
        }
    }
}

AircraftController::AircraftController()
    : target_altitude(2000.0), target_heading(60), target_speed(65),
      should_run(true) {
    last_time = std::chrono::high_resolution_clock::now();
}

double AircraftController::getTargetAltitude() const {
    return target_altitude;
}

double AircraftController::getTargetHeading() const {
    return target_heading;
}

double AircraftController::getTargetSpeed() const {
    return target_speed;
}

double AircraftController::calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double bearing = atan2(y, x) * 180.0 / M_PI;
    return fmod((bearing + 360.0), 360.0);  // Normalize to 0-360
}
    

double AircraftController::calculateCrossTrackError(double currentLat, double currentLon, 
                                Waypoint start, Waypoint end) {
    // to radians
    double lat1 = start.latitude * M_PI / 180.0;
    double lon1 = start.longitude * M_PI / 180.0;
    double lat2 = end.latitude * M_PI / 180.0;
    double lon2 = end.longitude * M_PI / 180.0;
    double lat3 = currentLat * M_PI / 180.0;
    double lon3 = currentLon * M_PI / 180.0;

    double d13 = acos(sin(lat1) * sin(lat3) + cos(lat1) * cos(lat3) * cos(lon1 - lon3));
    double bearing13 = calculateBearing(start.latitude, start.longitude, currentLat, currentLon) * M_PI / 180.0;
    double bearing12 = calculateBearing(start.latitude, start.longitude, end.latitude, end.longitude) * M_PI / 180.0;

    return asin(sin(d13) * sin(bearing13 - bearing12)) * 20902985.0; // Earth radius in ft
}

double AircraftController::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    lon2 = lon2 * M_PI / 180.0;

    // Haversine formula
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double earthRadius = 20902985.0; // Earth's radius in feet (6371000 m * 3.28084)
    return earthRadius * c;
}

void AircraftController::setFlightPath(const std::vector<Waypoint>& path) {
    flightPath = path;
    currentWaypointIndex = 0;
}

void AircraftController::updatePosition(double lat, double lon) {
    currentLatitude = lat;
    currentLongitude = lon;
}

void AircraftController::trackLineSegment() {
    if (flightPath.empty() || currentWaypointIndex >= flightPath.size()-1) {
        return;  
    }

    Waypoint currentWP = flightPath[currentWaypointIndex];
    Waypoint nextWP = flightPath[currentWaypointIndex + 1];

    double desiredHeading = calculateBearing(currentLatitude, currentLongitude, 
                                            nextWP.latitude, nextWP.longitude);
    
    // Calculate cross-track error for correction
    double crossTrackError = calculateCrossTrackError(currentLatitude, currentLongitude, 
                                                    currentWP, nextWP);
    
    // adjust heading based on cross-track error
    double correction = -crossTrackError / 1000.0;  
    double correctedHeading = fmod(desiredHeading + correction + 360.0, 360.0);

    static double previousHeading = correctedHeading; // Retain last heading
    double distanceToNext = calculateDistance(currentLatitude, currentLongitude, 
                                              nextWP.latitude, nextWP.longitude);
    
    // Look-ahead: Start turning when within a certain distance
    const double lookAheadDistance = 350.0; // ft, adjust based on speed
    const double smoothingFactor = 0.1;      // 0 to 1, lower = smoother but slower

    if (distanceToNext < lookAheadDistance && currentWaypointIndex + 1 < flightPath.size() - 1) {
        // Preview the next segment
        Waypoint futureWP = flightPath[currentWaypointIndex + 2];
        double nextHeading = calculateBearing(currentLatitude, currentLongitude, 
                                              futureWP.latitude, futureWP.longitude);
        
        // Interpolate between current and next heading based on distance
        double transitionRatio = (lookAheadDistance - distanceToNext) / lookAheadDistance;
        transitionRatio = std::max(0.0, std::min(transitionRatio, 1.0)); 
        
        double headingDiff = nextHeading - correctedHeading;
        if (headingDiff > 180.0) headingDiff -= 360.0;
        if (headingDiff < -180.0) headingDiff += 360.0;
        correctedHeading = correctedHeading + headingDiff * transitionRatio;
        correctedHeading = fmod(correctedHeading + 360.0, 360.0);
    }

    
    correctedHeading = previousHeading + smoothingFactor * (correctedHeading - previousHeading);
    correctedHeading = fmod(correctedHeading + 360.0, 360.0);
    previousHeading = correctedHeading;
    
    std::cout << "-------------Line Segment Tracking----------------" << std::endl;
    std::cout << "Waypoint Index: " << currentWaypointIndex << std::endl;
    std::cout << "Going to Heading: " << correctedHeading << std::endl;

    setTargetHeading(correctedHeading);
    setTargetAltitude(nextWP.altitude);
    setTargetSpeed(nextWP.speed);

    std::cout << "Distance to Waypoint (ft): " << distanceToNext << std::endl;
    if (distanceToNext < 500.0) {  // threshold [ft]
        std::cout << "REACHED WAYPOINT, GOING to NEXT" << std::endl;
        currentWaypointIndex++;
    }
}