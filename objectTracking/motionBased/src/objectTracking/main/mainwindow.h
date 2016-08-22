#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include <QFileDialog>
#include "../yzbxLib/qyzbxlib.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <dlib/svm_threaded.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/data_io.h>
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
typedef dlib::scan_fhog_pyramid<dlib::pyramid_down<6> > image_scanner_type;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_inputPath_clicked();

    void on_pushButton_bgs_clicked();

    void on_pushButton_tracking_clicked();

    void on_pushButton_recordReplay_clicked();

    void on_comboBox_dataset_currentIndexChanged(const QString &arg1);

    void on_comboBox_replay_currentIndexChanged(const QString &arg1);

    void on_comboBox_bgsType_currentIndexChanged(const QString &arg1);

    void on_comboBox_video_currentIndexChanged(const QString &arg1);

    void on_pushButton_stopBgs_clicked();

    void on_pushButton_stopTracking_clicked();

    void on_pushButton_detect_clicked();

private:
    Ui::MainWindow *ui;
    void loadIni(QString filepath);
    QString absoluteFilePath(QString currentPathOrFile, QString fileName);
    QStringList globalVideosList;
    QString globalVideoHome;
    QStringList globalDatasetList;
    QString globalAnnotationHome;
    QStringList globalAnnotationList;
    QStringList globalDetectionModelList;
    bool globalInited=false;
    bool globalDatasetChanged=false;
    TrackingStatus globalTrackingStatus;
    boost::property_tree::ptree globalPt;
    Tracking_yzbx *globalTracker=NULL;
};

#endif // MAINWINDOW_H
