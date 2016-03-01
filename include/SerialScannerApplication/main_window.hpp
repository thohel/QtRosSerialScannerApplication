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
#include <opencv/cv.hpp>
#include <opencv/cv.h>

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
    void processPicture();

public Q_SLOTS:
    void on_button_refresh_topic_clicked(bool check);
    void on_button_subscribe_topic_clicked(bool check);
    void updateView(int i);
    void updateTopics(QStringList list);

Q_SIGNALS:
    void getTopics();
    void subscribeToImage(QString topic);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace SerialScannerApplication

#endif // SerialScannerApplication_MAIN_WINDOW_H
