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

    // Signals for topic list in drop-down box
    QObject::connect(this, SIGNAL(getTopics()), &qnode, SLOT(findTopics()));
    QObject::connect(&qnode, SIGNAL(sendTopics(QStringList)), this, SLOT(updateTopics(QStringList)));

    // Signals for subscribing to image topic
    QObject::connect(this, SIGNAL(subscribeToImage(QString)), &qnode, SLOT(subscribeToImage(QString)));

    // Signal that we have chosen to use opencv camera grabber
    QObject::connect(ui.opencvCheckBox, SIGNAL(toggled(bool)), this, SLOT(opencvCheckBox_changed(bool)));

    ui.xposSlider->setEnabled(false);
    ui.yposSlider->setEnabled(false);
    ui.widthSlider->setEnabled(false);
    ui.heightSlider->setEnabled(false);
    setlocale (LC_NUMERIC, "C");

    // Init ROS
    qnode.init();
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

void MainWindow::updateTopics(QStringList list)
{
    ui.comboBox->clear();
    ui.comboBox->addItems(list);
}

void MainWindow::opencvCheckBox_changed(bool check)
{
    if (check) {
        // open the default camera, use something different from 0 otherwise;
        // Check VideoCapture documentation.
        if(!cap.open(0)) {
            std::cout << "Failed to open camera!" << std::endl;
            return;
        }

        timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(updateViewVoid()));

        // XXX: Select one of the two options
        if (false) {
            std::cout << cap.set(CV_CAP_PROP_FRAME_WIDTH, 2304) << std::endl;
            std::cout << cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1536) << std::endl;
            std::cout << cap.set(CV_CAP_PROP_FPS, 2) << std::endl;
            timer->start(500);
            std::cout << "Timer (for opencv) has been started" << std::endl;
        } else if (false) {
            std::cout << cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920) << std::endl;
            std::cout << cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080) << std::endl;
            std::cout << cap.set(CV_CAP_PROP_FPS, 5) << std::endl;
            timer->start(200);
            std::cout << "Timer (for opencv) has been started" << std::endl;
        } else {
            std::cout << cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920) << std::endl;
            std::cout << cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080) << std::endl;
            std::cout << cap.set(CV_CAP_PROP_FPS, 30) << std::endl;
            timer->start(33);
            std::cout << "Timer (for opencv) has been started" << std::endl;
        }
    } else {
        if (timer) {
            timer->stop();
            delete timer;
        }
    }
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

cv::Mat MainWindow::getThresholdImage(cv::Mat input, int threshold)
{
    cv::Mat return_mat = input;

    if (input.type() != CV_8UC1) {
        // Convert the image to grayscale
        cvtColor(input, return_mat, CV_BGR2GRAY);
    }

    cv::Mat return_mat_binary;
    cv::threshold(return_mat, return_mat_binary, threshold, 255, 0);
    return return_mat_binary;
}

cv::Mat MainWindow::getSecondOrderDerivativeOfImage(cv::Mat input)
{
    int kernel_size = 3;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    // Convert the image to grayscale
    cv::Mat input_gray;
    cv::cvtColor(input, input_gray, CV_BGR2GRAY);

    // Apply Laplacian transformation
    cv::Mat return_mat;
    cv::Laplacian(input_gray, return_mat, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT);

    // Change format
    cv::Mat return_mat_formatted;
    return_mat.convertTo(return_mat_formatted, CV_8UC3);

    return return_mat_formatted;
}

cv::Mat MainWindow::getContours(cv::Mat input)
{
    cv::vector<cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;

    cv::findContours(input, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Draw contours
    cv::Mat final = cv::Mat::zeros(input.size(), CV_8UC1 );

    for(uint i = 0; i < contours.size(); i++) {
        cv::Scalar color = cv::Scalar(255);
        cv::drawContours(final, contours, i, color, 1, 4, hierarchy, 2, cv::Point() );
    }

    return final;
}

cv::Mat MainWindow::getErodedImage(cv::Mat input, int erosion_size)
{
    cv::Mat element = getStructuringElement(cv::MORPH_RECT,
                                            cv::Size(2*erosion_size + 1, 2*erosion_size + 1),
                                            cv::Point(erosion_size, erosion_size));
    cv::Mat return_mat;

    // Apply the erosion operation
    cv::erode(input, return_mat, element);

    return return_mat;
}

cv::Mat MainWindow::getGaussianBlurSharpenedImage(cv::Mat input)
{
    cv::Mat temp;
    cv::Mat return_mat;
    cv::GaussianBlur(input, temp, cv::Size(0, 0), 3);
    cv::addWeighted(input, 3, temp, -1, 0, return_mat);
    return return_mat;
}

cv::Mat MainWindow::getMedianFilteredImage(cv::Mat input, int kernel_size)
{
    int kernelSize = kernel_size;

    if (kernelSize % 2 != 1)
        kernelSize = kernelSize - 1;

    if (kernelSize < 1)
        kernelSize = 1;

    cv::Mat return_mat;

    cv::medianBlur(input, return_mat, kernelSize);

    return return_mat;
}

void MainWindow::processPicture(cv::Mat pic)
{
    cv::Mat final = pic;
    cv::Mat temp;

    if (ui.upsampleCheckBox->isChecked()) {
        cv::pyrUp(final, temp, cv::Size(final.cols*2, final.rows*2));
        final = temp;
    }

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
        roi.width = (roi.x + ui->widthSlider->value() > final.cols) ? final.cols - roi.x : ui->widthSlider->value();
        roi.height = (roi.y + ui->heightSlider->value() > final.rows) ? final.rows - roi.y : ui->heightSlider->value();
        cv::rectangle(final, roi, cv::Scalar(0, 255, 0));
    }

    // The picture has been set but the cropping sliders are not activated
    // We now know the size of the image, so we can adjust our sliders accordingly
    if (!ui.xposSlider->isEnabled()) {
        ui.xposSlider->setMinimum(0);
        ui.xposSlider->setMaximum(final.cols);
        ui.xposSlider->setEnabled(true);

        ui.yposSlider->setMinimum(0);
        ui.yposSlider->setMaximum(final.rows);
        ui.yposSlider->setEnabled(true);

        ui.widthSlider->setMinimum(0);
        ui.widthSlider->setMaximum(final.cols);
        ui.widthSlider->setEnabled(true);

        ui.heightSlider->setMinimum(0);
        ui.heightSlider->setMaximum(final.rows);
        ui.heightSlider->setEnabled(true);
    }

    if (ui.sharpeningCheckBox->isChecked()) {
        final = getGaussianBlurSharpenedImage(final);
    }

    if (ui.medianCheckBox->isChecked()) {
        // Post-processing to remove noise
        final = getMedianFilteredImage(final, ui.medianSlider->value());
    }

    // Should we run the monochrome filtering?
    if (ui.monoChromeCheckBox->isChecked()) {
        final = getThresholdImage(final, ui.monochromeSlider->value());
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
        tessApi->Recognize(0);
        tesseract::ResultIterator *ri = tessApi->GetIterator();
        tesseract::PageIteratorLevel level = tesseract::RIL_WORD;
        if (ri != 0) {
            const char *word = ri->GetUTF8Text(level);
            QString ret = QString(word);
            ret.append("    Conf: ");
            char *conf_char;
            sprintf(conf_char, "%f", ri->Confidence(level));
            ret.append(conf_char);

            ui.foundTextLabel->setText(ret);

            delete[] word;
        }
        delete ri;
        delete tessApi;
    }

    QImage qImg = mat2qimage(final);
    QPixmap pixMap = QPixmap::fromImage(qImg);
    this->ui.imageLabel->setPixmap(pixMap);
}

void MainWindow::updateViewVoid()
{
    updateView(0);
}

void MainWindow::updateView(int i)
{
    std::cout << "Calling update-view" << std::endl;
/*
    if (qnode.pictureHasBeenSet()) {
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

        // Release the mutex and allow to process another image
        qnode.picLock.unlock();

        processPicture(cvImage.get()->image);
    }
*/
    if (ui.opencvCheckBox->isChecked()) {
        cv::Mat pic;
        std::cout << "Grabbing an opencv image" << std::endl;
        cap >> pic;

        if (pic.empty())
            return;

        std::cout << "Calling process picture with a opencv image" << std::endl;
        processPicture(pic);
    }
}

}  // namespace SerialScannerApplication

