#pragma once
#include <optional> //1
#include <string>
#include <vector>

namespace arl {

struct RoverPose {
    double worldX; 
    double worldY; 
    double headingDegrees;//ccw
};
// numeric, euc range>0 <max range, confidence 0->1
struct Detection {
    std::string id;
    double forward;//behind - front +
    double left;//right - left +
    double confidence;// 0->1
};

struct SafetyConfig {
    double minimumConfidence;
    double maximumRangeMeters;
    double laneHalfWidthMeters;
    double reactionTimeSeconds;
    double maximumDecelerationMps2;
};

struct Obstacle {
    std::string id;
    double forward;
    double left;
    double worldX;
    double worldY;
    double range;
};

std::vector<Obstacle> processDetections(
    const std::vector<Detection>& detections,
    const RoverPose& pose,
    const SafetyConfig& config);

std::optional<Obstacle> findNearestObstacle(const std::vector<Obstacle>& obstacles);

double calculateStoppingDistance(double speedKph, const SafetyConfig& config);


bool shouldEmergencyBrake(
    const std::vector<Obstacle>& obstacles,
    double speedKph,
    const SafetyConfig& config);

} //namespace arl