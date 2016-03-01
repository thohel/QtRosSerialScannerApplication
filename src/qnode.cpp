/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/SerialScannerApplication/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace SerialScannerApplication {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    pictureSet = false;
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"SerialScannerApplication");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	start();
	return true;
}

void QNode::run() {
	while ( ros::ok() ) {
		ros::spinOnce();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::findTopics()
{
    QStringList list;
    QString tmp;
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        //std::cout << "Topic : " << it - master_topics.begin() << ": " << info.name << " -> " << info.datatype <<       std::endl;
        tmp = QString::fromUtf8(info.datatype.c_str());

        // Add more types if needed
        if(QString::compare(tmp, "sensor_msgs/Image", Qt::CaseInsensitive) == 0   ) {
            list.append(QString::fromUtf8(info.name.c_str()));
        }
    }
    Q_EMIT sendTopics(list);
}

void QNode::subscribeToImage(QString topic)
{
    ros::NodeHandle n;
    const char *tmp = topic.toUtf8().constData();
    imageSub = n.subscribe<sensor_msgs::Image, QNode>(tmp, 1, &QNode::imageCallback, this);
}

// ********* CALLBACK METHODS ********* //
// ************************************ //
void QNode::imageCallback(const sensor_msgs::ImageConstPtr &image_msg)
{
    imagePtr = image_msg;
    pictureSet = true;
    updateView(0);
}

bool QNode::pictureHasBeenSet()
{
    return pictureSet;
}

}  // namespace SerialScannerApplication
