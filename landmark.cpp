#include <iostream>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using namespace gtsam;

int main()
{
    NonlinearFactorGraph graph;
    Symbol x1('x', 1), x2('x', 2), x3('x', 3), l1('l', 1), l2('l', 2);
    Pose2 priorMean(0.0, 0.0, 0.0);
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.add(PriorFactor<Pose2>(x1, priorMean, priorNoise));
    Pose2 odometryMean(2.0, 0.0, 0.0);
    auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    graph.add(BetweenFactor<Pose2>(x1, x2, odometryMean, odometryNoise));
    graph.add(BetweenFactor<Pose2>(x2, x3, odometryMean, odometryNoise));
    auto rot1 = Rot2::fromDegrees(45), rot2 = Rot2::fromDegrees(90), rot3 = Rot2::fromDegrees(90);
    auto bearRangeNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.2));
    graph.add(BearingRangeFactor<Pose2, Point2>(x1, l1, rot1, sqrt(8), bearRangeNoise));
    graph.add(BearingRangeFactor<Pose2, Point2>(x2, l1, rot2, 2.0, bearRangeNoise));
    graph.add(BearingRangeFactor<Pose2, Point2>(x3, l2, rot3, 2.0, bearRangeNoise));
    graph.print("\nFactor Graph:\n");
    
    Values initialEstimate;
    initialEstimate.insert(x1, Pose2(0.5, 0.0, 0.2));
    initialEstimate.insert(x2, Pose2(2.3, 0.1, -0.2));
    initialEstimate.insert(x3, Pose2(4.1, 0.1, 0.1));
    initialEstimate.insert(l1, Point2(1.8, 2.1));
    initialEstimate.insert(l2, Point2(4.1, 1.8));

    LevenbergMarquardtOptimizer optimzer(graph, initialEstimate);
    Values result = optimzer.optimize();
    result.print("\nFinal result\n");

    return 0;
}