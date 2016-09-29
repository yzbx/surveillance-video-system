#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    globalInited=false;
    ui->setupUi(this);
    bgsFactory_yzbx bgsFac;
    QStringList list=bgsFac.getBgsTypeList();
    list.sort();
    ui->comboBox_bgsType->addItems(list);

    trackingFactory_yzbx trackingFac;
    QStringList tlist=trackingFac.getTrackingTypeList();
    tlist.sort();
    ui->comboBox_trackingType->addItems(tlist);

    ui->lineEdit_inputPath->setReadOnly(true);
    ui->lineEdit_inputPath->setEnabled(false);

    //    globalDatasetList<<"Multi-CameraTracking"<<"MOT2D2015-Train"<<"MOT2D2015-Test"
    //                    <<"VisualTrackerBenchmark"<<"UserDefine"<<"UrbanTrackerDataset";

    globalDatasetList<<"Multi-CameraTracking"<<"UrbanTrackerDataset"<<"UserDefine";

    globalDatasetList.sort();
    ui->comboBox_dataset->addItems(globalDatasetList);

    QString configFilePath="/home/yzbx/config/surveillance-video-system.ini";
    ui->lineEdit_inputPath->setText(configFilePath);

    loadIni(configFilePath);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_inputPath_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open config file"), "",
                                                    tr("ini (*.ini)"));

    //    QString fileName = QFileDialog::getExistingDirectory(this, tr("Open Input Directory"),
    //                                                         ".",
    //                                                         QFileDialog::ShowDirsOnly
    //                                                         | QFileDialog::DontResolveSymlinks);
    if(!fileName.isEmpty()){
        ui->lineEdit_inputPath->setText(fileName);

        QFileInfo finfo(fileName);
        if(finfo.exists()){
            globalDatasetChanged=true;
            loadIni(fileName);
            globalDatasetChanged=false;
        }
        else{
            qDebug()<<"cannot find file "<<fileName;
            exit(-1);
        }

    }
}

void MainWindow::on_pushButton_bgs_clicked()
{
    cv::destroyAllWindows();

    QString bgsType=ui->comboBox_bgsType->currentText();
    bgsFactory_yzbx bgsFac;
    QString videoFilePath=ui->comboBox_video->currentText();
    if(videoFilePath.compare("all")!=0){
        QString inputPath=globalVideoHome+"/"+videoFilePath;
        bgsFac.process(bgsType,inputPath);
    }
    else{
        int n=globalVideosList.length();

        for(int i=0;i<n;i++){
            QString inputPath=globalVideoHome+"/"+globalVideosList.at(i);
            bgsFac.process(bgsType,inputPath);
        }
    }

}

void MainWindow::loadIni(QString filepath)
{
    boost::property_tree::ini_parser::read_ini(filepath.toStdString(),globalPt);


    QString Dataset=QString::fromStdString(globalPt.get<std::string>("General.Dataset"));
    if(globalInited){
        Dataset=ui->comboBox_dataset->currentText();
        globalPt.put("General.Dataset",Dataset.toStdString());
        boost::property_tree::ini_parser::write_ini(filepath.toStdString(),globalPt);
    }
    else{
        QString DatasetTxt=QString::fromStdString(globalPt.get<std::string>("General.DatasetTxt"));
        if(!DatasetTxt.isEmpty()){
            QString filedata;
            QFile file;
            file.setFileName(DatasetTxt);
            if (!file.open(QFile::ReadOnly | QFile::Text)) {
                qDebug()<<"cannot open file "<<DatasetTxt;
                exit(-1);
            } else {
                QTextStream in(&file);
                filedata=in.readAll();
                globalDatasetList=filedata.split("\n",QString::SkipEmptyParts);

                ui->comboBox_dataset->clear();
                ui->comboBox_dataset->addItems(globalDatasetList);
                file.close();
            }
        }
    }

    assert(Dataset=="Multi-CameraTracking"||Dataset=="UrbanTrackerDataset"||Dataset=="UserDefine");


    QString VideoHome=QString::fromStdString(globalPt.get<std::string>(Dataset.toStdString()+".VideoHome"));
    QString VideoTxt=QString::fromStdString(globalPt.get<std::string>(Dataset.toStdString()+".VideoTxt"));

    VideoHome=absoluteFilePath(filepath,VideoHome);
    VideoTxt=absoluteFilePath(filepath,VideoTxt);

    QString filedata;
    QFile file;
    file.setFileName(VideoTxt);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        qDebug()<<"cannot open file "<<VideoTxt;
        exit(-1);
    } else {
        QTextStream in(&file);
        filedata=in.readAll();
        globalVideosList=filedata.split("\n",QString::SkipEmptyParts);
        globalVideoHome=VideoHome;
        file.close();
    }

    QString AnnotationHome=QString::fromStdString(globalPt.get<std::string>(Dataset.toStdString()+".AnnotationHome"));
    if(!AnnotationHome.isEmpty()){
        QString AnnotationTxt=QString::fromStdString(globalPt.get<std::string>(Dataset.toStdString()+".AnnotationTxt"));
        AnnotationHome=absoluteFilePath(filepath,AnnotationHome);
        AnnotationTxt=absoluteFilePath(filepath,AnnotationTxt);

        file.setFileName(AnnotationTxt);
        if (!file.open(QFile::ReadOnly | QFile::Text)) {
            qDebug()<<"cannot open file "<<AnnotationTxt;
            exit(-1);
        } else {
            QTextStream in(&file);
            filedata=in.readAll();
            globalAnnotationList=filedata.split("\n",QString::SkipEmptyParts);
            globalAnnotationHome=AnnotationHome;
            file.close();
        }
    }

    QString RecordHome=QString::fromStdString(globalPt.get<std::string>(Dataset.toStdString()+".RecordHome"));
    if(!RecordHome.isEmpty()){
        QString RecordTxt=QString::fromStdString(globalPt.get<std::string>(Dataset.toStdString()+".RecordTxt"));
        RecordHome=absoluteFilePath(filepath,RecordHome);
        RecordTxt=absoluteFilePath(filepath,RecordTxt);

        file.setFileName(RecordTxt);
        if (!file.open(QFile::ReadOnly | QFile::Text)) {
            qDebug()<<"cannot open file "<<RecordTxt;
            exit(-1);
        } else {
            QTextStream in(&file);
            filedata=in.readAll();
            globalRecordList=filedata.split("\n",QString::SkipEmptyParts);
            globalRecordHome=RecordHome;
            file.close();
        }
    }

    QString DetectionModelTxt=QString::fromStdString(globalPt.get<std::string>("Detection.DetectionModelTxt"));
    DetectionModelTxt=absoluteFilePath(filepath,DetectionModelTxt);
    file.setFileName(DetectionModelTxt);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        qDebug()<<"cannot open file "<<DetectionModelTxt;
        exit(-1);
    } else {
        QTextStream in(&file);
        filedata=in.readAll();
        globalDetectionModelList=filedata.split("\n",QString::SkipEmptyParts);
        file.close();
    }

    if(!globalInited){
        ui->comboBox_detection->clear();
        ui->comboBox_detection->addItems(globalDetectionModelList);
    }

    if(!globalInited||globalDatasetChanged){
        ui->comboBox_video->clear();
        ui->comboBox_video->addItems(globalVideosList);
        ui->comboBox_video->addItem("all");
    }

    if(!globalInited){
        QString bgsType=QString::fromStdString(globalPt.get<std::string>("General.BGSType"));
        ui->comboBox_bgsType->setCurrentText(bgsType);
    }

    if(ui->comboBox_replay->currentText()=="record"){
        ui->comboBox_replayFiles->clear();
        ui->comboBox_replayFiles->addItems(globalRecordList);
    }
    else{
        ui->comboBox_replayFiles->clear();
        ui->comboBox_replayFiles->addItems(globalAnnotationList);
    }

    if(!globalInited){
        ui->comboBox_dataset->setCurrentText(Dataset);
        globalInited=true;
    }
}

//BUG how to deal with fileName=../output ?
QString MainWindow::absoluteFilePath(QString currentPathOrFile, QString fileName)
{
    QFileInfo pinfo(currentPathOrFile);
    QString currentPath=pinfo.absolutePath();


    QFileInfo info;
    info.setFile(QDir(currentPath),fileName);
    if(info.isAbsolute()){
        return info.absoluteFilePath();
    }
    else{
        if(info.isDir()){
            QDir dir(currentPath);
            dir.cd(fileName);
            return dir.absolutePath();
        }
        else if(info.isFile()){
            QDir dir(currentPath);
            return dir.absoluteFilePath(fileName);
        }
        else{
            qDebug()<<"error in absoluteFilePath: "<<currentPathOrFile<<fileName;
            return "error";
        }
    }
}

void MainWindow::on_pushButton_tracking_clicked()
{
    //    cv::destroyAllWindows();
    if(globalTracker!=NULL){
        globalTracker->stop();
        cv::destroyAllWindows();
        if(!globalTracker->isFinished()){
            qDebug()<<"previous tracking not finished!!!";
            return;
        }

        delete globalTracker;
        globalTrackingStatus.bgsInited=false;
        if(globalTrackingStatus.ibgs!=NULL) delete globalTrackingStatus.ibgs;
    }
    QString trackType=ui->comboBox_trackingType->currentText();
    QString configFile=ui->lineEdit_inputPath->text();


    trackingFactory_yzbx trackerFac;
    Tracking_yzbx *tracker=trackerFac.getTrackingAlgorithm(trackType);

    globalTracker=tracker;

    QString videoFilePath=ui->comboBox_video->currentText();
    if(videoFilePath.compare("all")!=0){
        QString inputPath=globalVideoHome+"/"+videoFilePath;
        //        tracker->process(configFile,inputPath);
        if(trackType.compare("default",Qt::CaseInsensitive)==0){
            tracker->process(configFile,inputPath);
        }
        else if(trackType.compare("UrbanTracker",Qt::CaseInsensitive)==0){
            tracker->process(configFile,inputPath);
        }
        else{
            tracker->process(configFile,inputPath,&globalTrackingStatus);
        }
    }
    else{
        int n=globalVideosList.length();

        for(int i=0;i<n;i++){
            QString inputPath=globalVideoHome+"/"+globalVideosList.at(i);
            if(trackType.compare("default",Qt::CaseInsensitive)==0||trackType.compare("UrbanTracker",Qt::CaseInsensitive)==0){
                tracker->process(configFile,inputPath);
            }
            else{
                tracker->process(configFile,inputPath,&globalTrackingStatus);
            }
        }
    }

}

void MainWindow::on_pushButton_recordReplay_clicked()
{
    QString ReplayHome;
    if(ui->comboBox_replay->currentText()=="record"){
        ReplayHome=globalRecordHome;
    }
    else{
        ReplayHome=globalAnnotationHome;
    }

    QString ReplayFile=ui->comboBox_replayFiles->currentText();
    int index=ui->comboBox_replayFiles->currentIndex();
    QString videoFile=globalVideosList[index];


    QString videoFilePath=globalVideoHome+"/"+videoFile;
    QString replayFilePath=ReplayHome+"/"+ReplayFile;

    QFileInfo info(replayFilePath);
    if(info.exists()){
        qDebug()<<" record file path is "<<replayFilePath<<videoFilePath;
        replay.process(videoFilePath,replayFilePath);
    }
    else{
        qDebug()<<"cannot find file "<<replayFilePath;
        qDebug()<<"please make sure RecordHome/AnnotationHome is right";
        qDebug()<<"and make sure record file have the same order with video file";
    }

}

void MainWindow::on_comboBox_dataset_currentIndexChanged(const QString &arg1)
{
    if(globalInited){
        qDebug()<<"load dataset "<<arg1;
        globalDatasetChanged=true;
        loadIni(ui->lineEdit_inputPath->text());
        globalDatasetChanged=false;
    }
}

void MainWindow::on_comboBox_replay_currentIndexChanged(const QString &arg1)
{
    if(globalInited){
        if(arg1=="record"){
            ui->comboBox_replayFiles->clear();
            ui->comboBox_replayFiles->addItems(globalRecordList);
        }
        else{
            ui->comboBox_replayFiles->clear();
            ui->comboBox_replayFiles->addItems(globalAnnotationList);
        }
    }
}

void MainWindow::on_comboBox_bgsType_currentIndexChanged(const QString &arg1)
{
    if(globalInited){
        qDebug()<<"bgs type: "<<arg1;
        globalPt.put("General.BGSType",arg1.toStdString());
        QString filepath=ui->lineEdit_inputPath->text();
        boost::property_tree::ini_parser::write_ini(filepath.toStdString(),globalPt);
    }
}

void MainWindow::on_comboBox_video_currentIndexChanged(const QString &arg1)
{
    if(globalInited){
        qDebug()<<"change videos"<<arg1;
    }
}

void MainWindow::on_pushButton_stopBgs_clicked()
{
    qDebug()<<"stop bgs wait to be implement!";
}

void MainWindow::on_pushButton_stopTracking_clicked()
{
    if(ui->pushButton_stopTracking->text()=="stop"){
        if(globalTracker==NULL){
            qDebug()<<"nothing to stop";
        }
        else{
            if(globalTracker->isRunning()){
                globalTracker->stop();
                ui->pushButton_stopTracking->setText("continue");
            }
            else{
                qWarning()<<"already stop!";
                globalTracker->start();
            }
        }
    }
    else{
        if(globalTracker==NULL){
            qDebug()<<"nothing to continue";
        }
        else{
            if(globalTracker->isRunning()){
                qWarning()<<"still running";
                globalTracker->stop();
            }
            else{
                globalTracker->start();
                ui->pushButton_stopTracking->setText("stop");
            }
        }
    }
}

void MainWindow::on_pushButton_detect_clicked()
{
    QString model=ui->comboBox_detection->currentText();
    dlib::object_detector<image_scanner_type> detector;
    //    deserialize("face_detector.svm") >> detector2;
    dlib::deserialize(model.toStdString()) >>detector;
    dlib::array<dlib::array2d<unsigned char> > images_test;

    QString videoFile=globalVideoHome+"/"+ui->comboBox_video->currentText();
    FrameInput frameinput;
    cv::Mat img_input;
    frameinput.getNextFrame(videoFile,img_input);
    dlib::image_window win;

    while(!img_input.empty()){
        dlib::cv_image<dlib::bgr_pixel> dimg(img_input);
        std::vector<dlib::rectangle> faces = detector(dimg);
        win.clear_overlay();
        win.set_image(dimg);
        win.add_overlay(faces, dlib::rgb_pixel(255,0,0));

        img_input.release();
        frameinput.getNextFrame(videoFile,img_input);
    }

}

void MainWindow::on_pushButton_pureTracking_clicked()
{
    QString video=ui->comboBox_video->currentText();
    if(video.compare("all")==0){
        for(int i=0;i<globalVideosList.size();i++){
            QString videoFile=globalVideosList[i];
            pureTrackingInit(videoFile);
            pureTrackingOne(videoFile);
        }
    }
    else{
        QString videoFile=video;
        pureTrackingInit(videoFile);
        pureTrackingOne(videoFile);
    }

}

void MainWindow::pureTrackingInit(QString videoFile){
    //init pure tracker

    //1. new bgs, tracker
    QString pureTrackingType=ui->comboBox_pureTracking->currentText();
    if(pureTracker!=NULL){
        if(pureTracker->isRunning()){
            qWarning()<<"still running!";
        }
        else if(pureTracker->isFinished()){
            qWarning()<<"finished!";
        }

        //wait for 10s
        qWarning()<<"wait for 10s";
        pureTracker->wait(10*1000);
        if(pureTracker->isRunning()){
            qWarning()<<"still running!";
        }
        else if(pureTracker->isFinished()){
            qWarning()<<"finished!";
        }

        pureTracker->deleteLater();
    }

    if(pureTrackingType=="HungarianBasedTracking"){
        pureTracker=new HungarianBasedTracking;
    }
    else if(pureTrackingType=="GraphBasedTracker"){
        pureTracker=new GraphBasedTracker;
    }
    else{
        assert(false);
    }

    if(ibgs!=NULL){
        delete ibgs;
    }

    QString bgsType=ui->comboBox_bgsType->currentText();
    bgsFactory_yzbx bgsFac;
    ibgs=bgsFac.getBgsAlgorithm(bgsType);

    //2. init input,bgs,tracker
    frameInput.init(globalVideoHome+"/"+videoFile);
    frameInput.initBgs(ibgs,10);

    QString outputFileName=videoFile;

    globalPureTrackingVideoFile=videoFile;
    videoFile=globalVideoHome+"/"+videoFile;
    QFileInfo info(videoFile);
    if(info.isDir()){
        outputFileName.replace("/","_");
        outputFileName+=".txt";
    }
    else if(info.isFile()){
        outputFileName.replace("/","_");
        outputFileName.replace(info.suffix(),"txt");
    }
    else{
        assert(false);
    }

    qDebug()<<"output filename: "<<outputFileName;
    pureTracker->init(outputFileName);
}

void MainWindow::pureTrackingOne(QString videoFile){
    assert(ibgs!=NULL);
    assert(pureTracker!=NULL);

    videoFile=globalVideoHome+"/"+videoFile;

    cv::Mat img_before;
    int FirstFG_frameNum=0;
    while(1){
        //1. get input frame
        cv::Mat img_fg,img_input,img_bg;
        frameInput.getNextFrame(videoFile,img_input);
        while(!img_input.empty()&&!img_before.empty()){
            if(yzbxlib::isSameImage(img_input,img_before)){
                qWarning()<<"the same image find!";
                frameInput.getNextFrame(videoFile,img_input);
            }
            else{
                break;
            }
        }
        img_before=img_fg;

        if(img_input.empty()||globalStop){
            break;
        }

        //2. detection and tracking
        ibgs->process(img_input,img_fg,img_bg);
        if(!img_fg.empty()){
            pureTracker->tracking(img_input,img_fg);
        }
        else{
            //BUG, we need record the frameNum for the first img_fg!!!
            FirstFG_frameNum++;
            pureTracker->FirstFG_frameNum=FirstFG_frameNum;
        }

        //3. process event of stop
        QApplication::processEvents();
    }
    globalStop=false;
}
void MainWindow::on_pushButton_globalStop_clicked()
{
    globalStop=true;
}

void MainWindow::on_pushButton_pureTrackingStop_clicked()
{
    assert(pureTracker!=NULL);
    if(pureTracker->isRunning()){
        globalStop=true;
    }
    else{
        pureTrackingOne(globalPureTrackingVideoFile);
    }
}

void MainWindow::on_pushButton_test_clicked()
{
    qDebug()<<"test";
    QString videoFile=globalVideoHome+"/"+ui->comboBox_video->currentText();
    VideoCapture cap(videoFile.toStdString());
    assert(cap.isOpened());
    Mat img_pre;
    while(1){
        Mat img_input;
        cap>>img_input;
        if(img_input.empty()){
            break;
        }

        img_pre=img_input;
        if(img_pre.empty()){
            continue;
        }

        Mat img1=img_pre;
        Mat img2=img_input;

        //        Ptr<FeatureDetector> detector=FeatureDetector::create("FAST");
        int minFeatureNum=50;
        int maxFeatureNum=1000;
        int maxIters=10;
        //        Ptr<FeatureDetector> detector=new DynamicAdaptedFeatureDetector ( new FastAdjuster(10,true), minFeatureNum, maxFeatureNum, maxIters);
        //        Ptr<FeatureDetector> detector=FeatureDetector::create("BRISK");
        //        Ptr<FeatureDetector> detector = new DynamicAdaptedFeatureDetector ( new FastAdjuster(10,true), 50, 100, 10);
        Ptr<FeatureDetector> detector=new BRISK(100);
        vector<KeyPoint> keypoints_1,keypoints_2;

        detector->detect(img1, keypoints_1);
        detector->detect(img2, keypoints_2);
        if(keypoints_1.empty()||keypoints_2.empty()){
            return;
        }

        //        cv::initModule_contrib();
        //        cv::initModule_features2d();
        Mat descriptors_1, descriptors_2;
        //        Ptr<DescriptorExtractor> extractor = new SurfDescriptorExtractor;
        Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("FREAK");
        //        Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("SURF");
        assert(!extractor.empty());

        Mat gray1,gray2;
        if(img1.channels()==3){
            cvtColor(img1,gray1,CV_BGR2GRAY);
            cvtColor(img2,gray2,CV_BGR2GRAY);
        }
        else{
            gray1=img1;
            gray2=img2;
        }
        imshow("gray1",gray1);
        imshow("gray2",gray2);
        qDebug()<<"keypoints_1: "<<keypoints_1.size();
        qDebug()<<"keypoints_2: "<<keypoints_2.size();
        extractor->compute( gray1, keypoints_1, descriptors_1 );
        extractor->compute( gray2, keypoints_2, descriptors_2 );

        vector< vector<DMatch> > matches;
        vector<DMatch> good_matches;
        int global_match_ratio=0.8;
        //        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
        Ptr<DescriptorMatcher> matcher=new BFMatcher(NORM_HAMMING,true);
        //        matcher->knnMatch( descriptors_1, descriptors_2, matches, 2 );

        //        for(size_t i = 0; i < matches.size(); i++) {
        //            DMatch first = matches[i][0];
        //            if(matches[i].size()>1){
        //                float dist1 = matches[i][0].distance;
        //                float dist2 = matches[i][1].distance;
        //                if(dist1 < global_match_ratio * dist2) {
        //                    good_matches.push_back(first);
        //                }
        //            }
        //            else{
        //                good_matches.push_back(first);
        //            }
        //        }
        matcher->match(descriptors_1,descriptors_2,good_matches);
        Mat outimg;
        cv::drawMatches(gray1,keypoints_1,gray2,keypoints_2,good_matches,outimg,Scalar(0,0,255),Scalar(0,255,255));
        imshow("outimg",outimg);
        waitKey(30);
    }
}

void MainWindow::convertCsvToImage(){
    QString ReplayHome;
    if(ui->comboBox_replay->currentText()=="record"){
        ReplayHome=globalRecordHome;
    }
    else{
        ReplayHome=globalAnnotationHome;
    }

    QString ReplayFile=ui->comboBox_replayFiles->currentText();
    int index=ui->comboBox_replayFiles->currentIndex();
    QString videoFile=globalVideosList[index];


    QString videoFilePath=globalVideoHome+"/"+videoFile;
    QString replayFilePath=ReplayHome+"/"+ReplayFile;

    VideoCapture cap(videoFilePath.toStdString());
    qDebug()<<"videoFile: "<<videoFilePath;
    qDebug()<<"ReplayFile: "<<replayFilePath;
    assert(cap.isOpened());
    Mat img_input;
    cap>>img_input;
    assert(!img_input.empty());
    cv::Scalar Colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 127, 255), cv::Scalar(127, 0, 255), cv::Scalar(127, 0, 127) };

    QFile file(replayFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        qDebug()<<"fail to open file "<<replayFilePath;
        return;
    }

    QTextStream in(&file);
    //object id set
    std::set<int> idSet;
    //lastObjectPosVector
    vector<pair<int,Point_t>> posVector;

    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList numstrList=line.split(",",QString::SkipEmptyParts);
        if(numstrList.size()<6){
            qDebug()<<"maybe end of file";
            break;
        }
        int frameNum=numstrList[0].toInt();
        int objectId=numstrList[1].toInt();
        Rect_t rect(numstrList[2].toFloat(),numstrList[3].toFloat(),
                numstrList[4].toFloat(),numstrList[5].toFloat());
        Point_t pos=(rect.br()+rect.tl());
        pos.x=pos.x/2.0;
        pos.y=pos.y/2.0;

        auto it=idSet.find(objectId);
        if(it==idSet.end()){
            idSet.insert(objectId);
            posVector.push_back(make_pair(objectId,pos));
        }
        else{
            for(auto p=posVector.begin();p!=posVector.end();p++){
                if(p->first==objectId){
                    cv::Scalar color=Colors[objectId % 9];
                    int thickness=1;
                    cv::line(img_input,p->second,pos,color,thickness);

                    //update pos
                    p->second=pos;
                    break;
                }
            }
        }

        if(frameNum%100==0){
            qDebug()<<"frameNum="<<frameNum<<",objectId="<<objectId;
        }
        //        imshow("trajectory",img_input);
        //        cv::waitKey(30);
    }

    imshow(ReplayFile.toStdString(),img_input);
    QString saveFile=ReplayFile;
    saveFile.replace(".csv",".jpg");
    imwrite(saveFile.toStdString(),img_input);
    cv::waitKey(1000);

    cap.release();
    cv::destroyAllWindows();
}

void MainWindow::on_pushButton_vibe_clicked()
{
    QString videoFile=ui->comboBox_video->currentText();
    QString videoFilePath=globalVideoHome+"/"+videoFile;
    QString bgsType=ui->comboBox_vibe->currentText();
    if(bgsType=="VIBE"){
        vibe::VIBE bgs(3);
        VideoCapture cap(videoFilePath.toStdString());
        assert(cap.isOpened());
        cv::Mat img_input,img_fg,img_bg;
        int frameNum=0;
        while(1){
            cap>>img_input;
            if(img_input.empty()){
                break;
            }

            cv::GaussianBlur(img_input, img_input, cv::Size(5,5), 1.5);

            if(frameNum==0){
                bgs.init(img_input);
            }
            else{
                bgs.update(img_input);
                img_fg=bgs.getMask();
                cv::medianBlur(img_fg, img_fg, 5);
                yzbxlib::showImageInWindow("img_input",img_input);
                yzbxlib::showImageInWindow("img_fg",img_fg);
            }

            cv::waitKey(30);
            frameNum++;
            if(frameNum%100==0){
                qDebug()<<"frameNum="<<frameNum;
            }
        }
    }
    else if(bgsType=="PBAS"){
        PBAS bgs;
        VideoCapture cap(videoFilePath.toStdString());
        assert(cap.isOpened());
        cv::Mat img_input,img_fg,img_bg;
        int frameNum=0;
        while(1){
            cap>>img_input;
            if(img_input.empty()){
                break;
            }

            cv::GaussianBlur(img_input, img_input, cv::Size(5,5), 1.5);

            bgs.process(&img_input, &img_fg);
            cv::medianBlur(img_fg, img_fg, 5);
            yzbxlib::showImageInWindow("img_input",img_input);
            yzbxlib::showImageInWindow("img_fg",img_fg);
            cv::waitKey(30);
            frameNum++;
            if(frameNum%100==0){
                qDebug()<<"frameNum="<<frameNum;
            }
        }
    }
    else if(bgsType=="SJN_MultiCueBGS"){
        bgsFactory_yzbx fac;
        IBGS *ibgs=fac.getBgsAlgorithm(bgsType);
        VideoCapture cap(videoFilePath.toStdString());
        assert(cap.isOpened());
        cv::Mat img_input,img_fg,img_bg;
        int frameNum=0;
        while(1){
            cap>>img_input;
            if(img_input.empty()){
                break;
            }

            cv::GaussianBlur(img_input, img_input, cv::Size(5,5), 1.5);

            ibgs->process(img_input, img_fg,img_bg);
            cv::medianBlur(img_fg, img_fg, 5);
            yzbxlib::showImageInWindow("img_input",img_input);
            yzbxlib::showImageInWindow("img_fg",img_fg);
            cv::waitKey(30);
            frameNum++;
            if(frameNum%100==0){
                qDebug()<<"frameNum="<<frameNum;
            }
        }
    }
    else{
        assert(false);
    }
}

void MainWindow::on_pushButton_vibeBasedTracking_clicked()
{
    QString videoFile=ui->comboBox_video->currentText();
    if(videoFile=="all"){
        for(int i=0;i<globalVideosList.size();i++){
            cv::destroyAllWindows();
            videoFile=globalVideosList[i];
            QString videoFilePath=globalVideoHome+"/"+videoFile;
            QString bgsType=ui->comboBox_vibe->currentText();

            PipeLineTracking tracker;
            qDebug()<<"bgsType is not used now! "<<bgsType;
            QString recordFile=yzbxlib::getOutputFileName(videoFile);
            tracker.setRecordFile(recordFile);
            tracker.process(videoFilePath);
        }
    }
    else{

        QString videoFilePath=globalVideoHome+"/"+videoFile;
        QString bgsType=ui->comboBox_vibe->currentText();

        PipeLineTracking tracker;
        qDebug()<<"bgsType is not used now! "<<bgsType;
        QString recordFile=yzbxlib::getOutputFileName(videoFile);
        tracker.setRecordFile(recordFile);
        tracker.process(videoFilePath);
    }

}

void MainWindow::on_pushButton_pipeLineTracking_clicked()
{
    QString inputData=globalVideoHome+"/"+ui->comboBox_video->currentText();
    PipeLineTracking tracker;
    tracker.process(inputData,"");
}

void MainWindow::on_pushButton_KLTTracking_clicked()
{
    QString videoFile=ui->comboBox_video->currentText();
    if(videoFile==""){

    }
    else{
        QString videoPath=globalVideoHome+"/"+videoFile;
        QString configFile=ui->lineEdit_inputPath->text();
        DataDrivePipeLine *klt=new KLTTrackingDemo(configFile);
        klt->mainData->setCurrentVideo(videoFile);
        klt->run();
    }
}

void MainWindow::on_pushButton_MeanShiftTracking_clicked()
{
    QString videoFile=ui->comboBox_video->currentText();
    if(videoFile=="all"){

    }
    else{
        QString videoPath=globalVideoHome+"/"+videoFile;
        QString configFile=ui->lineEdit_inputPath->text();
        DataDrivePipeLine *klt=new MeanShiftTrackingDemo(configFile);
        klt->mainData->setCurrentVideo(videoFile);
        klt->run();
    }
}

void MainWindow::on_pushButton_camshiftTracking_clicked()
{
    QString videoFile=ui->comboBox_video->currentText();
    if(videoFile=="all"){

    }
    else{
        QString videoPath=globalVideoHome+"/"+videoFile;
        QString configFile=ui->lineEdit_inputPath->text();
        DataDrivePipeLine *klt=new CamShiftTrackingDemo(configFile);
        klt->mainData->setCurrentVideo(videoFile);
        klt->run();
    }
}

void MainWindow::on_pushButton_dataDrive_clicked()
{
    QString videoFile=ui->comboBox_video->currentText();
    QString configFile=ui->lineEdit_inputPath->text();
    std::unique_ptr<DataDrivePipeLine> tracker=std::make_unique<DataDriveTracker>(DataDriveTracker(configFile));

    if(videoFile=="all"){
        tracker->runAll();
    }
    else{
        tracker->mainData->setCurrentVideo(videoFile);
        tracker->run();
    }
}
