#include "robot_state_publisher/robot_state_publisher.hpp"

int main(int argc, char** argv) {
    ArgumentParser parser("robot_state_publisher", "Publishes TF messages");
    parser.add<std::string>("file", "Robot description file");
    parser.add<std::string>("ip", "RIX Hub IP Address", 'i', "127.0.0.1");
    parser.add<std::string>("topic", "Joint state topic", 'T', "/joint_states");
    parser.add<std::string>("tf_pub_ep", "Endpoint for TF broadcaster to host on", {});
    parser.add<double>("rate", "Publish rate in rate", 'r', 20.0);
    parser.add<bool>("ignore_timestamps", "Ignore the timestamp of joint state messages", 't', false);

    if (!parser.parse(argc, argv)) {
        Log::error << parser.help() << std::endl;
        return 1;
    }

    std::string robot_description_file;
    if (!parser.get<std::string>("file", robot_description_file)) {
        Log::error << "Failed to get jrobot argument" << std::endl;
        return 1;
    }

    rix::ipc::Endpoint rixhub_endpoint;
    if (!parser.get<std::string>("ip", rixhub_endpoint.address)) {
        Log::error << "Failed to get ip argument" << std::endl;
        return 1;
    }
    rixhub_endpoint.port = RIXHUB_PORT;

    std::string topic;
    if (!parser.get<std::string>("topic", topic)) {
        Log::error << "Failed to get ip argument" << std::endl;
        return 1;
    }

    rix::ipc::Endpoint tf_endpoint("127.0.0.1", 0);
    std::string endpoint_string;
    if (!parser.get<std::string>("tf_pub_ep", endpoint_string)) {
        rix::util::Log::error << "Failed to get pub_ep argument" << std::endl;
        return 1;
    }
    if (!endpoint_string.empty()) {
        size_t colon_pos = endpoint_string.find_first_of(':');
        tf_endpoint.address = endpoint_string.substr(0, colon_pos);
        tf_endpoint.port = std::stoi(endpoint_string.substr(colon_pos));
    }

    double rate;
    if (!parser.get<double>("rate", rate)) {
        Log::error << "Failed to get rate argument" << std::endl;
        return 1;
    }

    bool ignore_timestamps;
    if (!parser.get<bool>("ignore_timestamps", ignore_timestamps)) {
        Log::error << "Failed to get ignore_timestamps argument" << std::endl;
        return 1;
    }

    auto robot = std::make_shared<rix::robot::RobotModel>(robot_description_file);
    auto rsp = std::make_shared<RobotStatePublisher>(rixhub_endpoint, robot, topic, rate, ignore_timestamps, tf_endpoint);

    auto signal = std::make_shared<rix::ipc::Signal>(SIGINT);
    rsp->spin(signal);

    return 0;
}