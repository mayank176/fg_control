#include <udp_io.h>

UDP_IO::UDP_IO(const std::string &receive_host, int receive_port,
        const std::string &send_host, int send_port) 
        : receive_host(receive_host), receive_port(receive_port), 
        send_host(send_host), send_port(send_port), 
        receive_socket(io_context), send_socket(io_context), is_initialized(false) {}

// Custom 64-bit network-to-host conversion
uint64_t ntohll(uint64_t val) {
    return ((uint64_t)ntohl(val & 0xFFFFFFFF) << 32) | ntohl(val >> 32);
}

// Helper function to swap bytes for a float value
float UDP_IO::swap_float_bytes(float value) {
    // This handles the conversion from network byte order (big-endian) to host byte order
    uint32_t* asInt = reinterpret_cast<uint32_t*>(&value);
    uint32_t swapped = ntohl(*asInt);
    return *reinterpret_cast<float*>(&swapped);
}


// Custom 64-bit host-to-network conversion
uint64_t htonll(uint64_t val) {
    return ((uint64_t)htonl(val & 0xFFFFFFFF) << 32) | htonl(val >> 32);
}
   

bool UDP_IO::sendFlightControls(const FlightControls &controls) {
    if (!is_initialized) {
        throw std::runtime_error("UDP interface not initialized");
    }

    try {
        // Convert to network byte order (big-endian)
        FlightControls network_controls;
        const uint64_t* src = reinterpret_cast<const uint64_t*>(&controls);
        uint64_t* dest = reinterpret_cast<uint64_t*>(&network_controls);
        for (int i = 0; i < 4; ++i) {  // 4 doubles
            dest[i] = htonll(src[i]);  // Host-to-network
        }

        size_t bytes_sent = send_socket.send_to(
            boost::asio::buffer(&network_controls, sizeof(network_controls)),
            send_endpoint
        );

        return (bytes_sent == sizeof(FlightControls));
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error sending data: " << e.what() << std::endl;
        return false;
    }
}

void UDP_IO::initialize() {
    try{ 
        boost::asio::ip::udp::endpoint receive_endpoint(boost::asio::ip::address_v4::any(), receive_port);       
        receive_socket.open(boost::asio::ip::udp::v4());
        receive_socket.bind(receive_endpoint);

        // Set socket timeout (100ms)
        boost::asio::socket_base::receive_buffer_size option_buffer(65536);
        receive_socket.set_option(option_buffer);
        
        #if BOOST_VERSION >= 106600
            // For Boost 1.66 and newer
            receive_socket.non_blocking(true);
        #else
            // For older Boost versions
            boost::asio::socket_base::non_blocking_io non_blocking_option(true);
            receive_socket.io_control(non_blocking_option);
        #endif

        send_socket.open(boost::asio::ip::udp::v4());
    
        // Resolve the send endpoint
        boost::asio::ip::udp::resolver resolver(io_context);
        auto endpoints = resolver.resolve(
            boost::asio::ip::udp::v4(),
            send_host, 
            std::to_string(send_port)
        );
        send_endpoint = *endpoints.begin();
        
        boost::asio::socket_base::reuse_address option(true);
        receive_socket.set_option(option);

        is_initialized = true;
        std::cout << "UDP interface initialized successfully." << std::endl;
    }
    catch (const boost::system::system_error& e) {
        std::cerr << "Error initializing UDP interface: " << e.what() << std::endl;
        throw;
    }
}

bool UDP_IO::receiveFlightState(FlightState &state) {
    if (!is_initialized) throw std::runtime_error("UDP not initialized!");

    try {
        boost::asio::ip::udp::endpoint sender_endpoint;
        boost::array<char, sizeof(FlightState)> recv_buffer;
        
        boost::system::error_code error;
        size_t bytes_received = receive_socket.receive_from(
            boost::asio::buffer(recv_buffer), sender_endpoint, 0, error
        );
        
        if (error == boost::asio::error::would_block) return false;
        if (error) {
            std::cerr << "Receive error: " << error.message() << std::endl;
            return false;
        }
        if (bytes_received != sizeof(FlightState)) {
            std::cerr << "Received " << bytes_received << " bytes, expected " 
                      << sizeof(FlightState) << std::endl;
            return false;
        }

        // Convert from network byte order (big-endian) to host (little-endian)
        auto* raw = reinterpret_cast<uint64_t*>(recv_buffer.data());
        uint64_t converted[8];
        for (int i = 0; i < 11; ++i) {
            converted[i] = ntohll(raw[i]);  // Convert each 64-bit double
        }
        std::memcpy(&state, converted, sizeof(FlightState));

        // Optional: Sanity check
        if (state.altitude_ft < -1000 || state.altitude_ft > 100000 ||
            state.heading_deg < -360 || state.heading_deg > 360 ||
            state.airspeed_kt < -10 || state.airspeed_kt > 2000) {
            std::cerr << "Invalid state: Alt=" << state.altitude_ft 
                      << ", Hdg=" << state.heading_deg 
                      << ", Spd=" << state.airspeed_kt << std::endl;
            return false;
        }

        return true;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error receiving data: " << e.what() << std::endl;
        return false;
    }
}

