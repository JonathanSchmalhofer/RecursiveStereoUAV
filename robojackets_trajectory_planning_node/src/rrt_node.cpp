#include <string>
#include <ros/ros.h>
/*
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QtQuick>
#include <QtWidgets>
#include <QQuickPaintedItem>
#include <iostream>
*/
#include <Eigen/Dense>
#include <memory> // for make_unique
#include "2dplane/GridStateSpace.hpp"
#include "BiRRT.hpp"
#include "2dplane/2dplane.hpp"
#include "planning/Path.hpp"




int main(int argc, char **argv)
{
    
    
    
    ros::init(argc, argv, "rrt_node");
    ros::NodeHandle node_handle;
    
    ////////////////////////////////////////////////////////////////////////////////////
    
    std::shared_ptr<RRT::GridStateSpace> _stateSpace;
    std::unique_ptr<RRT::BiRRT<Eigen::Vector2d>> _biRRT;

    Eigen::Vector2d _startVel, _goalVel;
    std::vector<Eigen::Vector2d> _previousSolution;
    
    Eigen::Vector2d size(800, 600);
    _stateSpace = std::make_shared<RRT::GridStateSpace>(size.x(), size.y(), 40, 30);
    _biRRT = std::make_unique<RRT::BiRRT<Eigen::Vector2d>>(_stateSpace, 2);

    //  setup birrt
    _biRRT->setStartState(size / 10);
    _biRRT->setGoalState(size / 2);
    _biRRT->setMaxStepSize(30);
    _biRRT->setGoalMaxDist(12);

    _startVel = Eigen::Vector2d(3, 0);
    _goalVel = Eigen::Vector2d(0, 3);
    
    ////////////////////////////////////////////////////////////////////////////////////
    
    ROS_INFO("Starting RRT  node");

    int numTimes = 50;

    ros::Rate node_rate(1); // 1.0 hz
    while (ros::ok())
    {
        for (int i = 0; i < numTimes; i++) {
            _biRRT->grow();
        }
        
        ROS_INFO("#%d with size() = %d", _biRRT->iterationCount(), _biRRT->getPath().size());

        //  store solution
        _previousSolution.clear();
        if (_biRRT->startSolutionNode() != nullptr)
        {
            ROS_INFO("   Found a solution");
            _previousSolution = _biRRT->getPath();
            RRT::SmoothPath(_previousSolution, *_stateSpace);
        }

        ros::spinOnce();
        node_rate.sleep();
    }

    return 0;
}
