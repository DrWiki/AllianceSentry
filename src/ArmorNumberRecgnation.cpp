//
// Created by nvidia on 19-2-22.
//

#include "ArmorNumberRecgnation.h"

ArmorNumberRecgnation::ArmorNumberRecgnation() {

    for(int i = 0;i<9;++i){
        std::stringstream TemplatePath;
        TemplatePath << "../others/data/NumberImage/template/"<<i<<".png";
        Template.Templates.push_back(cv::imread(TemplatePath.str(),CV_LOAD_IMAGE_GRAYSCALE));
    }
}

int ArmorNumberRecgnation::RecgnizeByImageSub(cv::Mat src) {

    assert(Template.Templates[0].rows==src.rows && Template.Templates[0].cols==src.cols && Template.Templates[0].channels()==1);
    src.convertTo(src,CV_32FC1);

    std::vector<double> diffs;
    for(int i = 0 ; i<Template.Templates.size() ; ++i){
        Template.Templates[i].convertTo(Template.Templates[i],CV_32FC1);

        double abs;
        double sumsrc;
        double sumTemplate;
        abs = src.dot(Template.Templates[i]);
        sumsrc = src.dot(src);
        sumTemplate = Template.Templates[i].dot(Template.Templates[i]);
        /*
        cv::absdiff(src,Template.Templates[i],abs);
        */
        double sum = abs/100.0;
        double lensrc = sqrt(sumsrc/100.0);
        double lenTemolate = sqrt(sumTemplate/100.0);
        diffs.push_back(sum/(lensrc*lenTemolate));
//        std::cout<<"sum: "<<sum<<std::endl;
//        std::cout<<"lensrc: "<<lensrc<<std::endl;
//        std::cout<<"lenTemolate: "<<lenTemolate<<std::endl;
//        std::cout<<sum/(lensrc*lenTemolate)<<std::endl;
    }

    int minid = 0;
    int maxid = 0;
    double min = 14*14*255;
    double max = 0;
    for(int i = 0;i<Template.Templates.size();++i){
        /*
        if(diffs[i]<min){
            min=diffs[i];
            minid = i;
        }
        */
        if(diffs[i]>max){
            max=diffs[i];
            maxid = i;
        }
    }

//    if(max>10){
        return maxid;
//    } else{
//        return -1;
//    }
}

void ArmorNumberRecgnation::RecgniseBySVM() {

}

void ArmorNumberRecgnation::RecgnizeByKNN() {

}

void ArmorNumberRecgnation::RecgnizeByCNN() {

}
