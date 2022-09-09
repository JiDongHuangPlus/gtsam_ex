#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <Eigen/Core>

using namespace gtsam;
using namespace std;

int main()
{
    NonlinearFactorGraph graph;
    Pose2 priorMean(0.0, 0.0, 0.0);
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));
    Pose2 odometryMean(2.0, 0.0, 0.0);
    auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    graph.add(BetweenFactor<Pose2>(1, 2, odometryMean, odometryNoise));
    graph.add(BetweenFactor<Pose2>(2, 3, odometryMean, odometryNoise));

    Values initial;
    initial.insert(1, Pose2(0.5, 0.0, 0.2));
    initial.insert(2, Pose2(2.3, 0.1, -0.2));
    initial.insert(3, Pose2(4.1, 0.1, 0.1));
    auto result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print();

    Marginals marginals(graph, result);
    cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
    cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
    cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

    return 0;
}