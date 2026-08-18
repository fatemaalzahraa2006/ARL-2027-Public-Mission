#include "safety_monitor.hpp"

#include <cmath>

namespace arl {
namespace {

bool isFinite(const Detection& detection) {
    return std::isfinite(detection.forward)
        && std::isfinite(detection.left)
        && std::isfinite(detection.confidence);
}

}  // namespace

std::vector<Obstacle> processDetections(
    const std::vector<Detection>& detections,
    const RoverPose& pose,
    const SafetyConfig& config) {
    std::vector<Obstacle> obstacles;
    const double headingRadians = pose.headingDegrees * 3.14159265358979323846 / 180.0; //4
    const double cosine = std::cos(headingRadians);
    const double sine = std::sin(headingRadians);

    for (std::size_t index = 0; index < detections.size(); ++index) { //6
        const auto& detection = detections[index];
        const double range = std::hypot(detection.forward, detection.left);
        const bool validConfidence = detection.confidence >= config.minimumConfidence //5
            && detection.confidence <= 1.0;

        const bool validRange = range > 0.0 && range <= config.maximumRangeMeters;

        if (!isFinite(detection) || !validConfidence || !validRange) {
            continue;
        }

        obstacles.push_back({
            detection.id,
            detection.forward,
            detection.left,
            pose.worldX + cosine * detection.forward - sine * detection.left, //7?
            pose.worldY + sine * detection.forward + cosine * detection.left,
            range,
        });
    }

    return obstacles;
}

// Returns the nearest obstacle, or std::nullopt if there are no obstacles.

std::optional<Obstacle> findNearestObstacle(const std::vector<Obstacle>& obstacles) {
    if (obstacles.empty()) {
        return std::nullopt;
    }

    const Obstacle* nearest = &obstacles.front();
    for (const auto& obstacle : obstacles) {
        if (obstacle.range < nearest->range) { //3
            nearest = &obstacle;
        }
    }

    return *nearest;
}

//functions

double calculateStoppingDistance(double speedKph, const SafetyConfig& config) {
    const double speedMps =  speedKph/3.6; //2
    const double reactionDistance = speedMps * config.reactionTimeSeconds;
    const double brakingDistance = speedMps * speedMps
        / (2.0 * config.maximumDecelerationMps2);
    return reactionDistance + brakingDistance;
}
// 1+ obst 0 met forw, 1+ 
// obst in lane corridor or boundary
// 1+ obst forw<= stopping distance
//8
bool shouldEmergencyBrake(const std::vector<Obstacle>& obstacles, 
    double speedKph, const SafetyConfig& config) {

    bool cond1,cond2,cond3;
    const double stoppingDistance = calculateStoppingDistance(speedKph, config);

    for (const auto& obstacle : obstacles) {
        cond1 = std::abs(obstacle.left) <= config.laneHalfWidthMeters;
        cond2 = obstacle.forward <= stoppingDistance;
        cond3 = obstacle.forward >= 0.0;

        if (cond1 && cond2 && cond3) {
            return true;
        }
    }
    return false;
}

}  // namespace arl