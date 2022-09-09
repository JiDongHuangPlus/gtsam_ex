#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <iostream>
#include <fstream>

using namespace gtsam;
using namespace std;

class measureFactor : public NoiseModelFactor1<Pose2>
{
    public:
    measureFactor(Key j, double x, double y, const SharedNoiseModel& model):
    NoiseModelFactor1<Pose2>(model, j), _x(x), _y(y){}
    Vector evaluateError(const Pose2& q, boost::optional<Matrix&> H = boost::none) const override
    {
        const Rot2& R = q.rotation();
        if (H)
        {
            (*H) = (gtsam::Matrix(2, 3) <<
            R.c(), -R.s(), 0.0,
            -R.s(), R.c(), 0.0).finished();
        }
        return (Vector(2) << q.x() - _x, q.y() - _y).finished();
    }
    public:
    double _x, _y;
};

int main()
{
    // ofstream fs("../result.txt");
    NonlinearFactorGraph graph;

    Pose2 priorMean(0.0, 0.0, 0.0);
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    // graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));
    Pose2 odometryMean(2.0, 0.0, 0.0);
    auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    graph.add(BetweenFactor<Pose2>(1, 2, odometryMean, odometryNoise));
    graph.add(BetweenFactor<Pose2>(2, 3, odometryMean, odometryNoise));

    auto unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));
    graph.add(boost::make_shared<measureFactor>(1, 0.0, 0.0, unaryNoise));
    graph.add(boost::make_shared<measureFactor>(2, 2.0, 0.0, unaryNoise));
    graph.add(boost::make_shared<measureFactor>(3, 4.0, 0.0, unaryNoise));
    graph.print("\nFactor Graph:\n");

    Values initial;
    initial.insert(1, Pose2(0.5, 0.0, 0.2));
    initial.insert(2, Pose2(2.3, 0.1, -0.2));
    initial.insert(3, Pose2(4.1, 0.1, 0.1));
    LevenbergMarquardtOptimizer optimizer(graph, initial);
    auto result = optimizer.optimize();
    result.print("\nFinal Result:\n");

    Marginals marginals(graph, result);
    cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl
    << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl
    << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;


    return 0;
}