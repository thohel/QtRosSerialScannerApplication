/**
 * @file /include/SerialScannerApplication/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SerialScannerApplication_QNODE_HPP_
#define SerialScannerApplication_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/Image.h>
#include <mutex>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace SerialScannerApplication {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
    bool init();
	void run();
    sensor_msgs::ImageConstPtr imagePtr;
    bool pictureHasBeenSet();
    void imageCallback(const sensor_msgs::ImageConstPtr& imagePtr);
    std::mutex picLock;

public Q_SLOTS:
    void findTopics();
    void subscribeToImage(QString topic);

Q_SIGNALS:
//	void loggingUpdated();
    void rosShutdown();
    void updateView(int i);
    void sendTopics(QStringList list);

private:
	int init_argc;
	char** init_argv;
    bool pictureSet;
    ros::Subscriber imageSub;
};

}  // namespace SerialScannerApplication

#endif /* SerialScannerApplication_QNODE_HPP_ */
