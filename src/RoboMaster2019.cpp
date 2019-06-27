//
// Created by nvidia on 19-1-16.
//
#include "RoboMaster2019.h"

union {
    sint yv;
    char subyv[2];
} datay,datap,datas;
bool receive = true;
short int a = 0;
short int olda = 0;


std::chrono::steady_clock::time_point time_point= std::chrono::steady_clock::now();
std::chrono::steady_clock::time_point start_time= std::chrono::steady_clock::now();
void ExternalThread(RoboMaster2019 *rm){
    bug("All Serial is OK ttYUSBX");
    while(receive){
            char dataReceive[10];
            rm->serial->readBuffer(dataReceive,10);
            unsigned short int rcr = (unsigned short int)dataReceive[1]+
                                     (unsigned short int)dataReceive[3]+
                                     (unsigned short int)dataReceive[5];
            // RenMingWu has tool me both modulus operation and division operation are surprisingly time-cost;
            // so utilize a tips he taught me, after he laughed at the statement writen by me % .
            // this method is use to replace the %
            //
            //while (rcr>0XFF){ rcr-=0XFF; }
            rcr = rcr % 255;
            if((unsigned char)dataReceive[9] == (unsigned char)rcr ||  1){

                /**/
                datay.subyv[0] = dataReceive[1];
                datay.subyv[1] = dataReceive[2];

                for(int i = 0;i<rm->PTZvSize-1;++i){
                    rm->PTZv[i] = rm->PTZv[i+1];
                }
                rm->PTZv[rm->PTZvSize-1] = datay.yv;
                //bug2("Data:*************",datay.yv)
                for(int i = 0;i<rm->PTZvSize-1;++i){
                    rm->PTZva+=rm->PTZv[i];
                }
                rm-> PTZva *= rm->PTZvSizeInv;
                rm->PTZva >>=  10;


                /**/
                datap.subyv[0] = dataReceive[3];
                datap.subyv[1] = dataReceive[4];

                for(int i = 0;i<rm->PTZvSize-1;++i){
                    rm->PTZvp[i] = rm->PTZvp[i+1];
                }

                rm->PTZvp[rm->PTZvSize-1] = datap.yv;
                //bug2("Data: dd",datap.yv)
                for(int i = 0;i<rm->PTZvSize-1;++i){
                    rm->PTZvap+=rm->PTZvp[i];
                }
                rm-> PTZvap *= rm->PTZvSizeInv;
                rm->PTZvap >>=  10;

                /**/
                datas.subyv[0] = dataReceive[5];
                datas.subyv[1] = dataReceive[6];
                for(int i=0;i<rm->PTZvSize-1;i++){
                    rm->Paww[i] = rm->Paww[i+1];
                }
                rm->Paww[rm->PTZvSize-1] = datas.yv;
                //bug3("Paww",rm->Paww[rm->PTZvSize-1],datas.yv)
                /**/
                datas.subyv[0] = dataReceive[7];
                datas.subyv[1] = dataReceive[8];
                for(int i=0;i<rm->PTZvSize-1;i++){
                    rm->Yaww[i] = rm->Yaww[i+1];
                }
                rm->Yaww[rm->PTZvSize-1] = datas.yv;

                rm->velocity4Bullet = int(dataReceive[0]);
            }else{
                bug("failed")
            }
            /*
            for(int i = 0;i<10;++i){
                //if(i==5 || i==6)
                bug2("  "+ std::to_string(i)+"  ",(unsigned char)dataReceive[i]+0);
            }
            */

    }
}

/**
 *
 *
 *
 * */
sint RoboMaster2019::BuffMode() {

    timerBase.Start();
    if(this->cap->camera.IndustrialCamera != nullptr){
        cap->camera.IndustrialCamera->SetExposureTime(1);
    }
    while(this->mode_Con_Vid_Pic!=-1){
        // Get a BGR Image(Original)
        this->cap->read(Frame);
        frameNum++;
        cv::resize(this->Frame,this->Frame,this->setter.resolution);
        SentryBgr2GrayBuff();
        BuffExpert();
        AutoCollimation2();
        /*
        BuffExpert_ImageSub();
        */
        EndOneLoop();
        KeyBoardCallBack();
        bug((frameNum*1024)/(timerBase.GetMs()));
    }
    return 222;
}

/**
 *
 *
 *
 * */

void RoboMaster2019::SentryBgr2GrayBuff() {
    cv::Mat dst = Frame.clone();
    // this is the hot point*****************************************************
    cv::cvtColor(dst, dst, CV_BGR2GRAY);
    cv::GaussianBlur(dst,dst,cv::Size(5,5),1,0);
    // CV_BGR2Lab is waste more time than CV_BGR2GRAY
    Gray = dst > setter.Threshold4Gray2BinaryPowerTrigger;
    cv::dilate(Gray,Gray,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5)));
    //cv::erode(Gray,Gray,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5)));
    SHOW_IMAGE1;
}
/**
 * Description : Power Switcher
 *
 *
 */
void RoboMaster2019::BuffExpert() {
    cv::Mat powerCopy = Gray.clone();
    cv::imshow("Power Tiger",powerCopy);
    cv::findContours(powerCopy,AllContours, cv::RETR_CCOMP/*cv::RETR_EXTERNAL*/, cv::CHAIN_APPROX_NONE);
    unsigned long int conSize0 = AllContours.size();
    std::vector<cv::Moments> moments(conSize0);
    std::vector<cv::RotatedRect> rects(conSize0);
    std::vector<cv::RotatedRect> ellipse(conSize0);
    analyze.AnalyzeTheMoment(AllContours);
    if(conSize0>0){
        for(int i = 0;i<conSize0;++i){
            // Using Moment Tech to Judge direction and shape and .....
            moments[i] = cv::moments(AllContours[i], false);
            // Using RotateRect to Choose Target and per
            rects[i] = cv::minAreaRect(AllContours[i]);

            cv::Point2f tempPoint[4];
            cv::Point2f tempPointEllipse[4];

            //TODO:There is a class named HuMoment in Opencv lib.
            double Mu2 = (moments[i].nu20-moments[i].nu02)*(moments[i].nu20-moments[i].nu02)+4*moments[i].nu11*moments[i].nu11;
            double Mu1 = moments[i].nu02+moments[i].nu20;
            if(abs(Mu1-0.37)<0.05){
                ellipse[i] = cv::fitEllipse(AllContours[i]);
                rects[i].points(tempPoint);
                ellipse[i].points(tempPointEllipse);

                cv::Point2f dir4ellipse((tempPointEllipse[0].x+tempPointEllipse[3].x)/2-(tempPointEllipse[1].x+tempPointEllipse[2].x)/2,
                                        (tempPointEllipse[0].y+tempPointEllipse[3].y)/2-(tempPointEllipse[1].y+tempPointEllipse[2].y)/2);
                cv::Point2f center = rects[i].center;
                cv::Point2f mcenter(static_cast<float >(moments[i].m10/moments[i].m00),
                                    static_cast<float>(moments[i].m01/moments[i].m00));
                cv::Point2f dir4Moment(mcenter.x-center.x,mcenter.y-center.y);

                if(dir4ellipse.x*dir4Moment.x+dir4ellipse.y*dir4Moment.y < 0){
                    dir4ellipse.x = -dir4ellipse.x;
                    dir4ellipse.y = -dir4ellipse.y;
                }

                cv::arrowedLine(Frame,
                                cv::Point(static_cast<int>(center.x-dir4ellipse.x),
                                          static_cast<int>(center.y-dir4ellipse.y)),
                                cv::Point(static_cast<int>(center.x+dir4ellipse.x),
                                          static_cast<int>(center.y+dir4ellipse.y)),
                                W,1);
                Predict = center;
                cv::line(Frame,tempPoint[0],tempPoint[1],B,1);
                cv::line(Frame,tempPoint[1],tempPoint[2],G,1);
                cv::line(Frame,tempPoint[2],tempPoint[3],R,1);
                cv::line(Frame,tempPoint[3],tempPoint[0],W,1);
                cv::putText(Frame," Target! ",tempPoint[3],1,1,B,1);

            }
        }
    }
    cv::imshow("Buff",Frame);
}


void ExternalSubMain(RoboMaster2019 *rm){
    while (RUN) {
        switch (rm->key) {
            case 0:
                rm->key = rm->ScoutMode();
                break;
            case 1:
                //rm->key = rm->BuffMode();
                rm->cap->read(rm->Frame);
                cv::resize(rm->Frame, rm->Frame, rm->setter.resolution);
                cv::imshow("IR", rm->Frame);
                cv::waitKey(1);
                break;
            case 2:
                rm->key = rm->BuffMode();
                break;
            default:
                rm->ReadThread->detach();
                return;
        }
    }
}


/*-----------------CONSTRUCTOR  ->  PUBLIC--------------------------*//*


     _   _        ____                ____        _     _ _
    | \ | | ___  / ___|___  _ __     |  _ \ _   _| |__ | (_) ___
    |  \| |/ _ \| |   / _ \| '_ \    | |_) | | | | '_ \| | |/ __|
    | |\  | (_) | |__| (_) | | | |   |  __/| |_| | |_) | | | (__
    |_| \_|\___/ \____\___/|_| |_|___|_|    \__,_|_.__/|_|_|\___|
                                |_____|


*//*-----------------CONSTRUCTOR  ->  PUBLIC--------------------------*/
//to Use Enter More Often not Space, Because ...

/* * *
 * Core Funtionality
 *
 *
 **/
void RoboMaster2019::SentryMode() {
    ReadThread = new std::thread(ExternalThread,this);
    MainThread = new std::thread(ExternalSubMain,this);
    MainThread->join();
    ReadThread->join();
    isAllOK = true;
    return;
}

/**
 *
 *
 *
 *
 * */
sint RoboMaster2019::ScoutMode() {
    timerBase.Start();
    while(this->mode_Con_Vid_Pic!=-1){
        if(!this->cap->read(Frame)){
            cap->camera.IndustrialCamera->stop();
            delete cap->camera.IndustrialCamera;
            delete cap;
            ReadThread->detach();
            MainThread->detach();
            return 90;
        }
        //SolveCloestPosition(naturalCoordinates*12.2535f+480.0f,1000,0);
        frameNum++;
        SolveCloestPosition(naturalCoordinates,2000,1);
        cv::resize(this->Frame,this->Frame,this->setter.resolution);
        SentryBgr2Gray();
        ArmorExpert();
        AutoCollimation2();

        KeyBoardCallBack();
        EndOneLoop();
        /*
        analyzeStatistics->ini(PTZva+200);
        analyzeStatistics->Ploti(analyzeStatistics->boardi,G);
        cv::imshow("ana",analyzeStatistics->boardi);
        analyzeStatistics->clear(analyzeStatistics->boardi);

        error->ini((int)FinallyTraget[TargetSize-1].x);
        error->Ploti(error->boardi,G);
        cv::imshow("anaerror",error->boardi);
        error->clear(error->boardi);
        */
        bug((frameNum*1024)/(timerBase.GetMs()));
    }
    return 111;
}


/*-----------------PUBLIC  ->  PRIVATE------------------*//*

             ____       _            _
            |  _ \ _ __(_)_   ____ _| |_ ___
            | |_) | '__| \ \ / / _` | __/ _ \
            |  __/| |  | |\ V / (_| | ||  __/
            |_|   |_|  |_| \_/ \__,_|\__\___|


*//*-----------------PUBLIC  ->  PRIVATE------------------*/


/**
 *
 *
 *
 * */
void RoboMaster2019::SentryBgr2Gray() {
    cv::cvtColor(Frame, Gray, CV_BGR2GRAY);// this is the hot point   // CV_BGR2Lab is waste more time than CV_BGR2GRAY

    GrayEnhancement = Gray.clone();
    GrayEnhancement = GrayEnhancement*setter.LinearGain;

    Gray = Gray > setter.Threshold4Gray2Binary;
    //cv::threshold(Gray, Gray, 0, 255, CV_THRESH_OTSU);
    // Assistant Statements
    SHOW_IMAGE1;
}

void RoboMaster2019::ArmorExpert(){
    //find all contours
    cv::findContours(Gray.clone(),AllContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    //FindArmor->FindFromContoursOptimize(Frame,AllContours,Armors,setter);
    FindArmor->FindFromContours(Frame,AllContours,Armors,setter);
    if(!Armors.empty()){
        maxPossibility =  Armors[0].possibility;
        maxPossibility_index = 0;
        for(int i = 1;i<Armors.size();++i){
            Armors[i].WriteLabelBGR(DrawingBoard_1);
            if(maxPossibility<Armors[i].possibility){
                maxPossibility_index = i;
                maxPossibility = Armors[i].possibility;
            }
        }
        for(int fi = 1;fi<TargetSize;++fi){
            FinallyTraget[fi-1].x = FinallyTraget[fi].x;
            FinallyTraget[fi-1].y = FinallyTraget[fi].y;
            cv::putText(DrawingBoard_1,
                        std::to_string(FinallyTraget[fi-1].x)+"$"+
                        std::to_string(FinallyTraget[fi-1].y),
                        cv::Point(25,10+(TargetSize-fi)*12),1,0.8,cv::Scalar(0,255,255),1);
        }
        FinallyTraget[TargetSize-1].x = static_cast<int>(Armors[maxPossibility_index].ArmorCenter.x);
        FinallyTraget[TargetSize-1].y = static_cast<int>(Armors[maxPossibility_index].ArmorCenter.y);

        // The Distmeasurement3.at<float>(0,0)ance value is not enough precise
        // But the focus distance is a fix value, so I assume it as precise
        // and then I can draw a conclusion that the process of camera calibration should have been more precise.
        PitchYawDistance = pnp->GetXYZ(Armors[maxPossibility_index]);
        /*if(PitchYawDistance.z>710 || PitchYawDistance.z<0){
            Armors.clear();
            for(int fi = 1;fi<TargetSize;++fi){
                FinallyTraget[fi-1].x = FinallyTraget[fi].x;
                FinallyTraget[fi-1].y = FinallyTraget[fi].y;
            }
            FinallyTraget[TargetSize-1].x = FinallyTraget[TargetSize-2].x;
            FinallyTraget[TargetSize-1].y = FinallyTraget[TargetSize-2].x;
        }*/
    }else{
        for(int fi = 1;fi<TargetSize;++fi){
            FinallyTraget[fi-1].x = FinallyTraget[fi].x;
            FinallyTraget[fi-1].y = FinallyTraget[fi].y;
        }
        FinallyTraget[TargetSize-1].x = FinallyTraget[TargetSize-2].x;
        FinallyTraget[TargetSize-1].y = FinallyTraget[TargetSize-2].x;
    }
}
void RoboMaster2019::SolveCloestPosition(float s,float dis,float yaw) {

    float S[5] = {1120.0f,1641.0f,3681.0f,04202.0f,5322.0f};

    float SentryX = 0;
    float SentryY = 0;

    float EnemyX = 0;
    float EnemyY = 0;

    float CloestX = 0.0f;
    float CloestY = 0.0f;

    float theta = 0;
    float Fai = 0;
    sint part = 0;

    for(float i:S){
        if(s>i){
            part++;
        }else{
            break;
        }
    }
    //std::cout<<part<<std::endl;
    if(s<=S[0]){
        part = 0;
    }else if(s>S[0] && s<=S[1]){
        part = 1;
    }else if(s>S[1] && s<=S[2]){
        part = 2;
    }else if(s>S[2] && s<=S[3]){
        part = 3;
    }else if(s>S[3] && s<=S[4]){
        part = 4;
    }
    //std::cout<<part<<std::endl;

    float m1[2][2] = {{0.0f,0.0f},{0.0f,0.0f}};
    float mb[2] = {0.0f,0.0f};
    float a,b,c,d;
    float k = 0.0f;

    theta = 0;
    Fai = (theta+PTZva)/57.3f;
    switch (part){
        case 0:
            SentryX = 0.0f;
            SentryY = s;
            break;
        case 1:
            SentryX = 0-410*(1-cos((s-S[0])/410));
            SentryY = S[0]+410*sin((s-S[0])/410);
            break;
        case 2:
            SentryX = 0-410*(1-cos((S[1]-S[0])/410))-(s-S[1])/1.732f;
            SentryY = S[0]+410*1.732f/2+(s-S[1])/2.0f;
            break;
        case 3:
            SentryX = 0-410*(1-cos((S[1]-S[0])/410))-(S[2]-S[1])/1.732f-(410*sin((s-S[2])/410+30/57.3f)-410*sin(30/57.3f));
            SentryY = S[0]+410*1.732f/2+(S[2]-S[1])/2.0f+(410*cos(30/57.3f)-410*cos((s-S[2])/410+30/57.3f));
            break;
        case 4:
            SentryX = 0-410*(1-cos((S[1]-S[0])/410))-(S[2]-S[1])/1.732f-(410*sin((S[3]-S[2])/410+30/57.3f)-410*sin(30/57.3f));
            SentryY = S[0]+410*1.732f/2+(S[2]-S[1])/2.0f+(410*cos(30/57.3f)-410*cos((s-S[2])/410+30/57.3f))+s-S[3];
            break;
        default:

            break;
    }
   // cv::circle(DrawingBoard_map,cv::Point(SentryX/10+setter.resolution.width/2,480-SentryY/10),2,Y,-1);
    EnemyX = SentryX - dis*cos(Fai);
    EnemyY = SentryY - dis*sin(Fai);
    //cv::circle(DrawingBoard_map,cv::Point(EnemyX/10+setter.resolution.width/2,480-EnemyY/10),2,Y,-1);
    //cv::imshow("DrawingBoard_map",DrawingBoard_map);
    // y = 0.577*x + 4062;
    // y = 0.577*x + 1049;
    // x + 0*y = 0 / 1241.34
    // -q3*x + 3y = 1555*3
    // x*sinF - y*cosF = x1*sinF - y1*cosF

    float temp = EnemyY-0.577f*EnemyX;
    if( temp < 1049){
        a = 1.0f;
        b = 0.0f;
        c = cos(Fai);
        d = sin(Fai);

        k=a*d-b*c;
        m1[0][0] = d/k;
        m1[0][1] = -b/k;
        m1[1][0] = -c/k;
        m1[1][1] = a/k;

        mb[0] = 0.0f;
        mb[1] = SentryX*sin(Fai) - SentryY*cos(Fai);
    }else if(temp >4062){
        a = 1.732f;
        b = 3.000f;
        c = cos(Fai);
        d = sin(Fai);

        k=a*d-b*c;
        m1[0][0] = d/k;
        m1[0][1] = -b/k;
        m1[1][0] = -c/k;
        m1[1][1] = a/k;

        mb[0] = 1241.34f;
        mb[1] = SentryX*sin(Fai) - SentryY*cos(Fai);
    }else{
        a = 1.732f;
        b = 3.000f;
        c = cos(Fai);
        d = sin(Fai);

        k=a*d-b*c;
        m1[0][0] = d/k;
        m1[0][1] = -b/k;
        m1[1][0] = -c/k;
        m1[1][1] = a/k;

        mb[0] = 1555.0f*3;
        mb[1] = SentryX*sin(Fai) - SentryY*cos(Fai);
    }

    CloestX = m1[0][0]*mb[0]+m1[0][1]*mb[1];
    CloestY = m1[1][0]*mb[0]+m1[1][1]*mb[1];

    temp = CloestY-0.577f*CloestX;
    float ds = 200*10.0f*sin(PTZva/57.3f) ;


    if( temp < 1049){
        s = S[0]/2;
    }else if(temp >4062){
        s = S[1]+(S[2]-S[1])/2;
    }else{
        s = S[3]+(S[4]-S[3])/2;
    }


    /*
    if( temp < 1049){
        s = sqrt(CloestX*CloestX+CloestY*CloestY);
    }else if(temp >4062){
        s = S[1]+sqrt((CloestX+250)*(CloestX+250)+(CloestY-1153)*(CloestY-1153));
    }else{
        s = S[3]+sqrt((CloestX+2267)*(CloestX+2267)+(CloestY-3006)*(CloestY-3006));
    }
    */
    /*
    float ds = PitchYawDistance.z*10.0f*sin(PTZva/57.3f) ;
     */
    if(s<=S[0]){
        part = 0;
    }else if(s>S[0] && s<=S[1]){
        part = 1;
    }else if(s>S[1] && s<=S[2]){
        part = 2;
    }else if(s>S[2] && s<=S[3]){
        part = 3;
    }else if(s>S[3] && s<=S[4]){
        part = 4;
    }
    //bug2("part",part);
    NewnaturalCoordinates = sint(s);
}
/**
 *
 *
 ************************
 * */
void RoboMaster2019::PredictTarget() {
    //bug(PitchYawDistance.z*sin(PTZvap/(5730.0)));
    //We don't predict targets if the targets appear suddenly or are changed suddenly;
    if(FinallyTraget[TargetSize-1].x==-1 ||
       sqrtf((FinallyTraget[TargetSize-1].x-FinallyTraget[TargetSize-2].x)*(FinallyTraget[TargetSize-1].x-FinallyTraget[TargetSize-2].x)+
             (FinallyTraget[TargetSize-1].y-FinallyTraget[TargetSize-2].y)*(FinallyTraget[TargetSize-1].y-FinallyTraget[TargetSize-2].y))>60.0f){
        isSudden = 20;
    }
    float lamuda = 179.0f;
    float  f = 6;
    cv::Point2f tempP(0,0);
    cv::Point2f tempY(0,0);
    if(isSudden==0){
        //Prediction Based on PTZva
        cv::Mat measurement3 = cv::Mat::zeros(2,1,CV_32FC1);
        measurement3.at<float>(0,0) = atan((FinallyTraget[TargetSize-1].x-320)/(lamuda*f))*57.29f+PTZva;
        measurement3.at<float>(1,0) = (float)(Yaww[PTZvSize-1])/100;
        cv::Mat result3 = KFy->predict(measurement3);
        tempY.x = tan((result3.at<float>(0,0)-PTZva)/57.29f)*f*lamuda+320;
        float deltaY=1.3f*(FinallyTraget[TargetSize-1].x-tempY.x);
        tempY.x = FinallyTraget[TargetSize-1].x+deltaY;
        tempY.y = (float)(0);
        tempY = UltimateKFy->predict(tempY);

        cv::Mat measurement4 = cv::Mat::zeros(2,1,CV_32FC1);
        measurement4.at<float>(0,0) = atan((FinallyTraget[TargetSize-1].y -240)/(lamuda*f))*57.27f+(float)PTZvap/100.0f;
        measurement4.at<float>(1,0) = (float)(Paww[PTZvSize-1])/100;
        cv::Mat result4 = KFp->predict(measurement4);
        tempP.x = tan((result4.at<float>(0,0)-(float)PTZvap/100.0f)/57.27f)*f*lamuda+240;
        float deltaP=3.0f*(FinallyTraget[TargetSize-1].y-tempP.x);
        tempP.x = FinallyTraget[TargetSize-1].y+deltaP;
        tempP.y = (float)(0);
        tempP = UltimateKFp->predict(tempP);
        Predict.x = tempY.x;
        Predict.y = tempP.x;
        }else {
            tempY.x = atan((FinallyTraget[TargetSize - 1].x - 320) / (lamuda * f)) * 57.29f + PTZva;
            tempY.y = (float) (Yaww[PTZvSize - 1]) / 100;
            KFy->predict(tempY);

            tempP.x = atan((FinallyTraget[TargetSize - 1].y - 240) / (lamuda * f)) * 57.27f + (float)PTZvap/100.0f;
            tempP.y = (float) (Paww[PTZvSize - 1]) / 100;
            KFp->predict(tempP);

            Predict.x = FinallyTraget[TargetSize - 1].x;
            Predict.y = FinallyTraget[TargetSize - 1].y;
            isSudden--;
        }

        if(!Armors.empty()){
            float th = PTZvap/(5729.0f);
            float Ta = tan(th);
            float Co = cos(th);
            float x = -PitchYawDistance.z*Co;
            float v = 1000*((velocity4Bullet*0.1f)+20);
            float y = tan(PTZvap/5729.0f)-4900*x*x/(v*v*Co*Co);
            float Wh = (Ta*x-y-(350/Co))/(1+Ta*Ta);
            Predict.y = Predict.y+Wh*(0.01f);
        }
    DRAW_IMAGE_PREDICT;
}

void RoboMaster2019::Makedata() {
    if(Armors.empty()){
        dataSend[0] = 0XFF; //
        dataSend[6] = 0X00; // Armor type
    }else{
        dataSend[0] = 0XFE;
        Datas.a = (sint)(Predict.x);
        dataSend[1] = Datas.subdata[0];
        dataSend[2] = Datas.subdata[1];

        Datas.a = (sint)(Predict.y);
        dataSend[3] = Datas.subdata[0];
        dataSend[4] = Datas.subdata[1];

        auto distance = (unsigned char)((int)(PitchYawDistance.z/10));

        dataSend[5] = distance;
        dataSend[6] = 0X01;

        Datas.a = (sint)(NewnaturalCoordinates);
        dataSend[7] = Datas.subdata[0];
        dataSend[8] = Datas.subdata[1];
    }
    //0xff: Sentry connot shoot
    //0xfe: Sentry can shoot
    //dataSend[0] = abs(Yaww[PTZvSize-1])>2000? (unsigned char)0XFE : (unsigned char)0XFF;
    //dataSend[0] = isSudden!=0? 0XFE : 0XFF;
    if(fabs((float)Yaww[PTZvSize-1])>10000){
        dataSend[0] = 0XFF;
    }
    dataSend[9] = (unsigned char)((int(dataSend[1])+dataSend[3]+dataSend[5]) % 0XFF);
}
void RoboMaster2019::AutoCollimation2() {
    if(setter.isSerialDebug){
        PredictTarget();
        Makedata();
        for(int i = 0;i<10;++i){
            serial->writeBuffer(dataSend+i,1);
        }
        if(setter.isShowFinallyImage){
            for(int i = 0;i<10;++i) {
                std::stringstream temp;
                temp<<int(*(dataSend+i));
                cv::putText(DrawingBoard_1,temp.str(),cv::Point(600,50+i*15),1,1,Y,1);
            }
        }
    }
}
/**
 *
 *
 *
 * */
inline void RoboMaster2019::EndOneLoop() {
    cv::circle(DrawingBoard_1,cv::Point(320,240),5,cv::Scalar(0,0,255),-1);
    SHOW_IMAGE2;
    Armors.clear();
    AllContours.clear();
    FindArmor->CorrectLED.clear();
    FindArmor->LedGroup.clear();
    FindArmor->CorrectLEDRotateRectangle.clear();

    if(setter.isShowFinallyImage){
        cv::imshow("DrawingBoard_1",DrawingBoard_1);
        DrawingBoard_1 = cv::Mat::zeros(DrawingBoard_1.size(),CV_8UC3);
        DrawingBoard_1 = cv::Mat::zeros(DrawingBoard_1.size(),CV_8UC3);
    }
}

/**
 *
 *
 *
 * */
inline void RoboMaster2019::KeyBoardCallBack() {
    if(this->mode_Con_Vid_Pic == 0){
        this->key_current_loop = cv::waitKey(this->setter.cvWaitKeyTime);
        //bug2("Time",this->setter.cvWaitKeyTime)
    }else if(this->mode_Con_Vid_Pic == 1){
        this->key_current_loop = cv::waitKey(this->setter.cvWaitKeyTime);
        if( video->isOpened()){
            if(setter.isSaveOriginal){
                video->write(this->Frame); bug("video->write(this->Frame);"+std::to_string(frameNum));
            }else if(setter.isSaveResultVideo){
                video->write(this->DrawingBoard_1); bug("video->write(this->DrawingBoard_1);"+std::to_string(frameNum));
            }
        }
    }else if(this->mode_Con_Vid_Pic == 2){
        this->key_current_loop = cv::waitKey(0);
    }
    //if you didn't press any key, mode succeed the old value.
    this->key_current_loop = this->key_current_loop==-1? this->key_previous:this->key_current_loop;
    if(this->key_current_loop == 27){
        this->mode_Con_Vid_Pic = -1;                                  // End
        ReadThread->detach();
        MainThread->detach();
    }else if(this->key_current_loop == 'c'){                          //Mode : Continue Frame without shooting a video.
        this->mode_Con_Vid_Pic = 0;
    }else if(this->key_current_loop == 'v'){                          //Mode : Write video.
        this->mode_Con_Vid_Pic = 1;
    }else if(this->key_current_loop == 'x'){                          //Mode : analyze the picture.
        this->mode_Con_Vid_Pic = 2;
        //cv::imwrite(getTimePicture(),Frame);  bug("cv::imwrite(getTimePicture(),Frame): "+std::to_string(frameNum));
    }else if(this->key_current_loop == 'z'){
        DrawingBoard_2 = cv::Mat::zeros(DrawingBoard_2.size(),CV_8UC3);
    }
    this->key_previous = this->key_current_loop;                      //Key become old key.
}

/**
 * Spend Too Much Time!!!!!
 *
 *
 * */
cv::Mat RoboMaster2019::PerspectiveTrans(cv::Mat src, cv::Point2f *scrPoints, cv::Point2f *dstPoints) {
        cv::Mat dst;
        cv::Mat Trans = getPerspectiveTransform(scrPoints, dstPoints);
        warpPerspective(src, dst, Trans, cv::Size(src.cols, src.rows), CV_INTER_CUBIC);
        return dst;
}

int RoboMaster2019::CalcGain(int n) {
    int res = 0;
    for(int i = 1;i<=n;++i){
        res += FinallyTraget[TargetSize-i].x-320;
    }
    return res/n;
}

float RoboMaster2019::CalcGainf(int n) {
    int res = 0;
    for(int i = 1;i<=n;++i){
        res += FinallyTraget[TargetSize-i].x-320;
    }
    return float(res)/float(n);
}

RoboMaster2019::~RoboMaster2019() {
    if(this->cap->isIndustrialCamera || cap->camera.IndustrialCamera != nullptr){
        delete cap;
    }
}


/*
    if(this->cap->camera.IndustrialCamera != nullptr){
        cap->camera.IndustrialCamera->SetExposureTime(frameNum % 15);
    }
*/

//if(key_current_loop == 'a'){
//// Under this "a" mode, I have designed a switch structure to manage the process of target prediction
//// Above all, in order to make the PTZ run more steadily, we try to avoid prediction as the prediction
//// always causes a dramatic vibration when the target's position move a lot in sudden.
//if(OpponentStatus == 0){
//OpponentStatus =1;
//}
//if(OpponentStatus == 1){
//if(FinallyTraget[TargetSize-1].x-320>50){
//Ignore = (sint)(abs(FinallyTraget[TargetSize-1].x-320));
//}
//KFa->Kf->statePost.at<float>(0,0) = FinallyTraget[TargetSize-1].x;
//KFa->Kf->statePost.at<float>(1,0) = FinallyTraget[TargetSize-1].y;
//float lamuda = 100.0f;
//if((FinallyTraget[TargetSize-1].x-320)*PTZva<=0){
//Predict.x = (FinallyTraget[TargetSize-1].x+tan(PTZva/573.0f)*6*lamuda);
//}else{
//Predict.x = (FinallyTraget[TargetSize-1].x)+2*tan(PTZva/573.0f)*6*lamuda;
//}
//Predict = KFa->predict(Predict);
//KFa->drawKalman(DrawingBoard_2);
//OpponentStatus = 2;
////timerBase4Collimation.Start();
//}else if(OpponentStatus == 2){
//// The next statement help figure out how many ticks should be filled into the counter.
//if(FinallyTraget[TargetSize-1].x-320>50){
//Ignore = (sint)(abs(FinallyTraget[TargetSize-1].x-320));
//}
//// We certainly do predict, actually just compensation so far in 20190416,
//// when and only when the counter is empty.
//// And, if we don't do predict this time, the counter will minus one.
//if(Ignore == 0){
//float lamuda = 100.0f;
//bug("aaaaaaaaaaaaaa")
////Prediction Based on PTZva
//if((FinallyTraget[TargetSize-1].x-320)*PTZva<=0){
//Predict.x = FinallyTraget[TargetSize-1].x+tan(PTZva/573.0f)*6*lamuda;
//}else{
//Predict.x = FinallyTraget[TargetSize-1].x+2.1875f*tan(PTZva/573.0f)*6*lamuda;
//}
////auto dt = float(timerBase4Collimation.GetMs())/1000.0f;
//float dt = 10.0f;
////Predict.x = FinallyTraget[TargetSize-1].x+tan(PTZva/573.0f)*6*lamuda+0*1000*(atan((FinallyTraget[TargetSize-1].x-320)/(179.0f*6))-
////       atan((FinallyTraget[TargetSize-2].x-320)/(179.0f*6)));
////bug2("T",timerBase4Collimation.GetMs());
//// bug((atan((FinallyTraget[TargetSize-1].x-320)/(179.0f*6))-
////atan((FinallyTraget[TargetSize-2].x-320)/(179.0f*6))))
//float dir = PTZva*dt+
//            (atanf((FinallyTraget[TargetSize-1].x-320)/(179.0f*6))-atanf((FinallyTraget[TargetSize-2].x-320)/(179.0f*6)))*57.3f;
//bug2("TargetAction: " ,PTZva+
//                       (atanf((FinallyTraget[TargetSize-1].x-320)/(179.0f*6))-atanf((FinallyTraget[TargetSize-2].x-320)/(179.0f*6)))*57.3f);
//
//RealAction = (sint)dir;
//
////                    dirS->ini((int)dir+200);
////                    dirS->Ploti(dirS->boardi,G);
////                    cv::imshow("dirS",dirS->boardi);
////                    dirS->clear(dirS->boardi);
//
//Predict = KFa->predict(Predict);
////timerBase4Collimation.Start();
//KFa->drawKalman(DrawingBoard_2);
//}else{
//Predict.x = FinallyTraget[TargetSize-1].x;
//Ignore--;
//}
//}
//if(setter.isShowFinallyImage){
//cv::imshow("kalman",DrawingBoard_2);
//}
//}

/*
 *
         if(key_current_loop == 's'){
            int gain = CalcGain(TargetSize)*4;
            int temp = 30;
            if((FinallyTraget[TargetSize-1].x-FinallyTraget[TargetSize-2].x)*5<temp){
                temp = (FinallyTraget[TargetSize-1].x-FinallyTraget[TargetSize-2].x)*5;
            }
            if(gain>80){
                gain = 80;
            }
            if(gain<-80){
                gain = -80;
            }

            Predict.x = FinallyTraget[TargetSize-1].x+gain+temp*0;
            cv::putText(DrawingBoard_1,std::to_string(gain),
                        cv::Point(200,200),1,1,C,1);

            Predict = KF->predict(Predict);
            KF->drawKalman(DrawingBoard_2);
            cv::imshow("kalman",DrawingBoard_2);
        }
*/
/*        time_point= std::chrono::steady_clock::now();
        Predict.x = atan((FinallyTraget[TargetSize-1].x-320)/(lamuda*f))*57.27f+PTZva;
        long time=std::chrono::duration_cast<std::chrono::microseconds>(time_point-start_time).count();
        start_time= std::chrono::steady_clock::now();
        Predict.y = 1000000*(Predict.x-PreDict_old.x)/(float(time));
        PreDict_old.x = Predict.x;

        Predict.y = (float)(Yaww[PTZvSize-1])/100;*/

//When Mu1  are  equal, Mu2 will be equal with high possibility?
//if(abs(Mu1-0.29)<0.05 && /*moments[i].m00>3500*/ abs(Mu2-0.03)<0.05){
//ellipse[i] = cv::fitEllipse(AllContours[i]);
//
//rects[i].points(tempPoint);
//ellipse[i].points(tempPointEllipse);
//float width = sqrtf((tempPointEllipse[0].x-tempPointEllipse[3].x)*(tempPointEllipse[0].x-tempPointEllipse[3].x)
//                    +(tempPointEllipse[0].y-tempPointEllipse[3].y)*(tempPointEllipse[0].y-tempPointEllipse[3].y));
//float length = sqrtf((tempPointEllipse[0].x-tempPointEllipse[1].x)*(tempPointEllipse[0].x-tempPointEllipse[1].x)
//                     +(tempPointEllipse[0].y-tempPointEllipse[1].y)*(tempPointEllipse[0].y-tempPointEllipse[1].y));
//float ratio = length/width;
//
//if(fabs(ratio-1.7f)<0.5f){
//cv::Point2f dir4ellipse((tempPointEllipse[0].x+tempPointEllipse[3].x)/2-(tempPointEllipse[1].x+tempPointEllipse[2].x)/2,
//                        (tempPointEllipse[0].y+tempPointEllipse[3].y)/2-(tempPointEllipse[1].y+tempPointEllipse[2].y)/2);
//cv::Point2f center = rects[i].center;
//cv::Point2f mcenter(static_cast<float >(moments[i].m10/moments[i].m00),
//                    static_cast<float>(moments[i].m01/moments[i].m00));
//cv::Point2f dir4Moment(mcenter.x-center.x,mcenter.y-center.y);
//
//if(dir4ellipse.x*dir4Moment.x+dir4ellipse.y*dir4Moment.y < 0){
//dir4ellipse.x = -dir4ellipse.x;
//dir4ellipse.y = -dir4ellipse.y;
//}
//
///*
//cv::Point2f Target[4] = { cv::Point2f(width, 0), cv::Point2f(width, length), cv::Point2f(0, length), cv::Point2f(0, 0) };
//cv::Mat Persbective = PerspectiveTrans(Frame, tempPoint, Target);
//cv::Rect PowerTriggerROI(Target[3],Target[1]);
//cv::Mat Roi = Persbective(PowerTriggerROI).clone();
//cv::imshow("ROI",Roi);
//cv::imwrite(getTimePicture(),Roi);
//if(setter.isShowFinallyImage){
//    cv::imshow("Perspective",Persbective);
//}
//*/
//
//cv::arrowedLine(Frame,cv::Point(static_cast<int>(center.x-dir4ellipse.x),static_cast<int>(center.y-dir4ellipse.y)),
//        cv::Point(static_cast<int>(center.x+dir4ellipse.x),static_cast<int>(center.y+dir4ellipse.y)),
//        cv::Scalar::all(255),1);
//cv::line(Frame,tempPoint[0],tempPoint[1],cv::Scalar(255,0,0),2);
//cv::line(Frame,tempPoint[1],tempPoint[2],cv::Scalar(0,255,0),2);
//cv::line(Frame,tempPoint[2],tempPoint[3],cv::Scalar(0,0,255),2);
//cv::line(Frame,tempPoint[3],tempPoint[0],cv::Scalar(255,255,255),2);
//cv::putText(Frame,std::to_string(ratio),center,1,1,cv::Scalar(255,0,0),1);
//cv::putText(Frame," Finished! ",tempPoint[3],1,1,cv::Scalar(255,0,0),1);
//}
//}