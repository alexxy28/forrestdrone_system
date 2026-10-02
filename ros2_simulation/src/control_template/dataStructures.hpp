#ifndef DATA_STRUCTURES_HPP
#define DATA_STRUCTURES_HPP

// Everything two or more nodes must agree on. Types used by one file stay in that file.

struct Waypoint{
    double x;
    double y;
};

enum class HuskyStatus { IDLE, EN_ROUTE, ARRIVED, FAILED };

/// The text published on topics::HUSKY_STATUS.
inline const char* toString(HuskyStatus status){
    switch (status) {
        case HuskyStatus::IDLE:     return "IDLE";
        case HuskyStatus::EN_ROUTE: return "EN_ROUTE";
        case HuskyStatus::ARRIVED:  return "ARRIVED";
        case HuskyStatus::FAILED:   return "FAILED";
    }
    return "UNKNOWN";
}

namespace topics {
inline constexpr char ROUTE[]        = "/mission/route";         // nav_msgs/Path, control -> husky
inline constexpr char HUSKY_STATUS[] = "/mission/husky_status";  // std_msgs/String, husky -> control
inline constexpr char CASUALTY[]     = "/mission/casualty";      // geometry_msgs/PointStamped, parrot -> control
inline constexpr char STATE[]        = "/mission/state";         // std_msgs/String, control -> all
}

#endif
