/**
 * @file /include/SerialScannerApplication/main_window.hpp
 *
 * @brief Qt based gui for SerialScannerApplication.
 *
 * @date November 2010
 **/
#ifndef SerialScannerApplication_MAIN_WINDOW_H
#define SerialScannerApplication_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <opencv2/opencv.hpp>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace SerialScannerApplication {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
	void closeEvent(QCloseEvent *event); // Overloaded function
    QImage mat2qimage(cv::Mat& mat);
    void processPicture(cv::Mat pic);

public Q_SLOTS:
    void on_button_refresh_topic_clicked(bool check);
    void on_button_subscribe_topic_clicked(bool check);
    void updateViewVoid();
    void updateView(int i);
    void updateTopics(QStringList list);
    void opencvCheckBox_changed(bool check);

Q_SIGNALS:
    void getTopics();
    void subscribeToImage(QString topic);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    cv::VideoCapture cap;
    QTimer *timer;
    int rest_counter;
    // Image manipulation functions
    cv::Mat getErodedImage(cv::Mat input, int erosion_size);
    cv::Mat getContours(cv::Mat input);
    cv::Mat getThresholdImage(cv::Mat input, int threshold);
    cv::Mat getGaussianBlurSharpenedImage(cv::Mat input);
    cv::Mat getMedianFilteredImage(cv::Mat input, int kernel_size);
    cv::Mat getSecondOrderDerivativeOfImage(cv::Mat input);
};

}  // namespace SerialScannerApplication

#endif // SerialScannerApplication_MAIN_WINDOW_H
