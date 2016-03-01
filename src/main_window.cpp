/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/SerialScannerApplication/main_window.hpp"
#include <cv_bridge/cv_bridge.h>
#include <tesseract/baseapi.h>
#include <locale.h>
#include <thread>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace SerialScannerApplication {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

	setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    QObject::connect(&qnode, SIGNAL(updateView(int)), this, SLOT(updateView(int)));
//    QObject::connect(ui.monochromeSlider, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
//    QObject::connect(ui.textDetectionCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateViewBool(bool)));
//    QObject::connect(ui.monoChromeCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateViewBool(bool)));
//    QObject::connect(ui.medianCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateViewBool(bool)));
//    QObject::connect(ui.sharpeningCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateViewBool(bool)));
//    QObject::connect(ui.imageInversionCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateViewBool(bool)));
//    QObject::connect(ui.cropingCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateViewBool(bool)));

    // Signals for topic list in drop-down box
    QObject::connect(this, SIGNAL(getTopics()), &qnode, SLOT(findTopics()));
    QObject::connect(&qnode, SIGNAL(sendTopics(QStringList)), this, SLOT(updateTopics(QStringList)));

    // Signals for subscribing to image topic
    QObject::connect(this, SIGNAL(subscribeToImage(QString)), &qnode, SLOT(subscribeToImage(QString)));

    ui.xposSlider->setEnabled(false);
    ui.yposSlider->setEnabled(false);
    ui.widthSlider->setEnabled(false);
    ui.heightSlider->setEnabled(false);
    setlocale (LC_NUMERIC, "C");

    // Init ROS
    qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

void MainWindow::updateTopics(QStringList list)
{
    ui.comboBox->clear();
    ui.comboBox->addItems(list);
}

void MainWindow::on_button_refresh_topic_clicked(bool check)
{
    Q_EMIT getTopics();
}

void MainWindow::on_button_subscribe_topic_clicked(bool check)
{
    if (ui.comboBox->currentText().length() != 0) {
        if (ui.comboBox->currentText().contains("image_color"))
            Q_EMIT subscribeToImage(ui.comboBox->currentText());
    }
}

QImage MainWindow::mat2qimage(cv::Mat& mat) {
    switch ( mat.type() ) {
    // 8-bit, 4 channel
    case CV_8UC4: {
        QImage image( mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB32 );
        return image;
    }

        // 8-bit, 3 channel
    case CV_8UC3: {
        QImage image( mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888 );
        return image.rgbSwapped();
    }

        // 8-bit, 1 channel
    case CV_8UC1: {
        static QVector<QRgb>  sColorTable;

        // only create our color table once
        if ( sColorTable.isEmpty() ) {
            for ( int i = 0; i < 256; ++i )
                sColorTable.push_back( qRgb( i, i, i ) );
        }

        QImage image( mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8 );

        image.setColorTable( sColorTable );

        return image;
    }

    default:
        std::cout << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << mat.type();
        break;
    }

    return QImage();
}

void MainWindow::processPicture()
{
    // Lock the mutex so that we can avoid queing up images
    qnode.picLock.lock();

    // Convert ROS image message to jpeg with opencv
    cv_bridge::CvImagePtr cvImage;
    try {
        cvImage = cv_bridge::toCvCopy(qnode.imagePtr, "8UC3");
    } catch (cv_bridge::Exception& e) {
        std::cout << "Failed conversion of the image" << std::endl;
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat final = cvImage.get()->image;
    cv::Mat temp;
    cv::Mat temp2;

    if (ui.cropingCheckBox->isChecked()) {
        // Define a box that is our Region of Interest
        cv::Rect roi;
        roi.x = ui.xposSlider->value();
        roi.y = ui.yposSlider->value();
        roi.width = ui.widthSlider->value();
        roi.height = ui.heightSlider->value();

        // Crop the original image to the defined ROI
        temp = final(roi);
        final = temp;
    } else if (ui.cropingVisualizeCheckBox->isChecked()) {
        // Define a box that is our Region of Interest
        cv::Rect roi;
        roi.x = ui.xposSlider->value();
        roi.y = ui.yposSlider->value();
        roi.width = ui.widthSlider->value();
        roi.height = ui.heightSlider->value();
        cv::rectangle(final, roi, cv::Scalar(0, 255, 0));
    }

    // The picture has been set but the cropping sliders are not activated
    // We now know the size of the image, so we can adjust our sliders accordingly
    if (!ui.xposSlider->isEnabled()) {
        ui.xposSlider->setMinimum(0);
        ui.xposSlider->setMaximum(final.rows);
        ui.xposSlider->setEnabled(true);
        ui.heightSlider->setMinimum(0);
        ui.heightSlider->setMaximum(final.rows);
        ui.heightSlider->setEnabled(true);
        ui.yposSlider->setMinimum(0);
        ui.yposSlider->setMaximum(final.cols);
        ui.yposSlider->setEnabled(true);
        ui.widthSlider->setMinimum(0);
        ui.widthSlider->setMaximum(final.cols);
        ui.widthSlider->setEnabled(true);
    }

    if (ui.sharpeningCheckBox->isChecked()) {
        cv::GaussianBlur(final, temp2, cv::Size(0, 0), 5);
        cv::addWeighted(final, 3, temp2, -1, 0, temp);
        final = temp;
    }

    if (ui.medianCheckBox->isChecked()) {
        // Post-processing to remove noise
        int kernelSize = ui.medianSlider->value();

        if (kernelSize % 2 != 1)
            kernelSize = kernelSize - 1;

        if (kernelSize < 1)
            kernelSize = 1;

        cv::medianBlur(final, temp, kernelSize);
        final = temp;
    }

    // Should we run the monochrome filtering?
    if (ui.monoChromeCheckBox->isChecked()) {
        cv::inRange(final, cv::Scalar(this->ui.monochromeSlider->value(), this->ui.monochromeSlider->value(), this->ui.monochromeSlider->value()),
                           cv::Scalar(255, 255, 255), temp);
        final = temp;
    }

    if (ui.imageInversionCheckBox->isChecked()) {
        cv::bitwise_not(final, temp);
        final = temp;
    }

    // Should we run text detection?
    if (ui.textDetectionCheckBox->isChecked()) {
        tesseract::TessBaseAPI *tessApi = new tesseract::TessBaseAPI();
        cv::Mat clone = final.clone();
        if (tessApi->Init("/usr/share/tesseract-ocr/tessdata/", NULL))
            std::cout << "Failed to init tesseract" << std::endl;
        tessApi->SetVariable("classify_bln_numeric_mode", "1");
        tessApi->SetPageSegMode(tesseract::PageSegMode(7));
        tessApi->SetImage((uchar*)clone.data, clone.size().width, clone.size().height, clone.channels(), clone.step1());
        const char* out = tessApi->GetUTF8Text();

        if (out)
            ui.foundTextLabel->setText(QString(out));

        //delete tessApi;
        delete [] out;
        delete tessApi;
    }

    QImage qImg = mat2qimage(final);
    QPixmap pixMap = QPixmap::fromImage(qImg);
    this->ui.pixLabel->setPixmap(pixMap);

    // Release the mutex and allow to process another image
    qnode.picLock.unlock();
}

void MainWindow::updateView(int i)
{
    if (qnode.pictureHasBeenSet()) {
        processPicture();
    }
}

}  // namespace SerialScannerApplication

