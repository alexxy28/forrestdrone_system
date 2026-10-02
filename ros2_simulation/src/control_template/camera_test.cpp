#include "thermalCamera.hpp"
#include "rgbCamera.hpp"

#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_test");

    ThermalCamera thermal(node.get(), "/parrot1/thermal_camera");
    RgbCamera rgb(node.get(), "/parrot1/camera/image", "/parrot1/camera/depth/image");

    // Spin for up to 30 s, until every stream has delivered a frame.
    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (std::chrono::steady_clock::now() < end && !(thermal.hasData() && rgb.hasData())) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    int result = 0;
    if (thermal.hasData()) {
        double lo, hi;
        cv::minMaxLoc(thermal.getTemperatures(), &lo, &hi);
        std::cout << "thermal: " << thermal.getTemperatures().cols << "x" << thermal.getTemperatures().rows
                  << " " << lo << ".." << hi << " C, stamp " << thermal.getStamp() << std::endl;
    } else {
        std::cout << "thermal: NO DATA" << std::endl;
        result = 1;
    }

    if (rgb.hasData()) {
        double lo, hi;
        cv::minMaxLoc(rgb.getDepth(), &lo, &hi);
        std::cout << "image:   " << rgb.getImage().cols << "x" << rgb.getImage().rows
                  << ", stamp " << rgb.getImageStamp() << std::endl;
        std::cout << "depth:   " << rgb.getDepth().cols << "x" << rgb.getDepth().rows
                  << " " << lo << ".." << hi << " m, stamp " << rgb.getDepthStamp() << std::endl;
    } else {
        std::cout << "rgb/depth: NO DATA" << std::endl;
        result = 1;
    }

    rclcpp::shutdown();
    return result;
}
