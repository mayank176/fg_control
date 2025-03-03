#include "flight_controller.h"
#include <iostream>
#include <string>
#include <csignal>
#include <thread>
#include <atomic>

std::atomic<bool> running(true);
std::unique_ptr<AircraftController> controller;


void signalHandler(int signal) {
    std::cout << "Shutting down controller..." << std::endl;
    running = false;
    if (controller) {
        controller->stop();
    }
}

void statusThread() {
    while (running) {
        if (controller) {
            controller->printStatus();
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

int main(int argc, char* argv[]) {
    
    std::signal(SIGINT, signalHandler);
    
    std::cout << "Autopilot and Line-Segment Tracking in FlightGear" << std::endl;
    std::cout << "=================================" << std::endl;
    
    // Default values
    double target_altitude = 2100.0;
    double target_heading = 60.0;
    double target_speed = 100.0;
    
    try {
        controller = std::make_unique<AircraftController>();
        controller->initialize();

        std::vector<Waypoint> path = {
            {64.01222222, -22.63391667, 3000.0, 100.0},
            {64.00791667, -22.67425, 3000.0, 100.0},  
            {63.96197222, -22.67447222, 3000.0, 100.0},
            {63.98133333, -22.57525, 3000.0, 120.0}  
        };

        controller->setFlightPath(path);
        controller->setTargetAltitude(target_altitude);
        controller->setTargetHeading(target_heading);
        controller->setTargetSpeed(target_speed);
        
        std::thread status_thread(statusThread);
        std::cout << "Controller active." << std::endl;        
      
        controller->run();
        
        running = false;
        if (status_thread.joinable()) {
            status_thread.join();
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}