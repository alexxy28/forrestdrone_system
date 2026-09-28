#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>
#include <numeric>

class ThermalCasualtyDetector : public rclcpp::Node
{
public:
    ThermalCasualtyDetector()
        : Node("thermal_casualty_detector")
    {
        declare_parameter<double>("threshold_percentile", 99.5);
        declare_parameter<int>("min_hot_pixels", 80);

        threshold_percentile_ =
            get_parameter("threshold_percentile").as_double();

        min_hot_pixels_ =
            get_parameter("min_hot_pixels").as_int();

        thermal_sub_ =
            create_subscription<sensor_msgs::msg::Image>(
                "/parrot1/thermal_camera",
                10,
                std::bind(
                    &ThermalCasualtyDetector::thermalCallback,
                    this,
                    std::placeholders::_1));

        detection_pub_ =
            create_publisher<std_msgs::msg::Bool>(
                "/trace/casualty_detected",
                10);

        pixel_pub_ =
            create_publisher<geometry_msgs::msg::PointStamped>(
                "/trace/casualty_pixel",
                10);

        RCLCPP_INFO(
            get_logger(),
            "TRACE thermal casualty detector started.");

        RCLCPP_INFO(
            get_logger(),
            "Listening to /parrot1/thermal_camera");
    }

private:

    void thermalCallback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg->encoding != "mono16")
        {
            RCLCPP_WARN(
                get_logger(),
                "Expected mono16 but received %s",
                msg->encoding.c_str());
            return;
        }

        const size_t pixel_count =
            static_cast<size_t>(msg->width) *
            static_cast<size_t>(msg->height);

        if (msg->data.size() < pixel_count * sizeof(uint16_t))
        {
            RCLCPP_WARN(
                get_logger(),
                "Thermal image data smaller than expected.");
            return;
        }

        std::vector<uint16_t> pixels(pixel_count);

        std::memcpy(
            pixels.data(),
            msg->data.data(),
            pixel_count * sizeof(uint16_t));

        // Remove invalid zero pixels.
        std::vector<uint16_t> valid;
        valid.reserve(pixel_count);

        for (uint16_t value : pixels)
        {
            if (value > 0)
                valid.push_back(value);
        }

        if (valid.empty())
            return;

        // Determine adaptive thermal threshold.
        std::vector<uint16_t> sorted = valid;
        std::sort(sorted.begin(), sorted.end());

        double percentile =
            std::clamp(threshold_percentile_, 0.0, 100.0);

        size_t threshold_index =
            static_cast<size_t>(
                (percentile / 100.0) *
                static_cast<double>(sorted.size() - 1));

        uint16_t threshold =
            sorted[threshold_index];

        uint16_t maximum =
            *std::max_element(
                valid.begin(),
                valid.end());

        double total_intensity = 0.0;
        double weighted_x = 0.0;
        double weighted_y = 0.0;

        size_t hot_count = 0;

        for (size_t y = 0; y < msg->height; ++y)
        {
            for (size_t x = 0; x < msg->width; ++x)
            {
                size_t index =
                    y * msg->width + x;

                uint16_t value =
                    pixels[index];

                if (value >= threshold && value > 0)
                {
                    double weight =
                        static_cast<double>(value);

                    total_intensity += weight;

                    weighted_x +=
                        static_cast<double>(x) * weight;

                    weighted_y +=
                        static_cast<double>(y) * weight;

                    ++hot_count;
                }
            }
        }

        bool detected =
            hot_count >=
            static_cast<size_t>(min_hot_pixels_);

        std_msgs::msg::Bool detection_msg;
        detection_msg.data = detected;

        detection_pub_->publish(
            detection_msg);

        ++frame_count_;

        if (!detected ||
            total_intensity <= 0.0)
        {
            return;
        }

        double centroid_x =
            weighted_x / total_intensity;

        double centroid_y =
            weighted_y / total_intensity;

        geometry_msgs::msg::PointStamped candidate;

        candidate.header =
            msg->header;

        candidate.point.x =
            centroid_x;

        candidate.point.y =
            centroid_y;

        // For now z stores the raw maximum
        // thermal intensity, NOT temperature.
        candidate.point.z =
            static_cast<double>(maximum);

        pixel_pub_->publish(candidate);

        if (frame_count_ % 15 == 0)
        {
            RCLCPP_INFO(
                get_logger(),
                "THERMAL CANDIDATE | "
                "pixel=(%.1f, %.1f) | "
                "hot_pixels=%zu | "
                "threshold=%u | max=%u",
                centroid_x,
                centroid_y,
                hot_count,
                threshold,
                maximum);
        }
    }

    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr
        thermal_sub_;

    rclcpp::Publisher<
        std_msgs::msg::Bool>::SharedPtr
        detection_pub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PointStamped>::SharedPtr
        pixel_pub_;

    double threshold_percentile_;
    int min_hot_pixels_;

    size_t frame_count_ = 0;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ThermalCasualtyDetector>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
