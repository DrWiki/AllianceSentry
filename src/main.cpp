#include "LedBar.h"
#include "Setter.h"
#include "Calibrator.h"
#include "RoboMaster2019.h"
#include "ArmorNumberRecgnation.h"
#include "AllianceVideoCapture.h"
/// /// /// /// /// ///
/// *This is the Entrance of our Alliance2019_Sentry
/// \param argv
/// \param argc
/// \return 0 when normally terminated and other numbers when abnormally
/// \description This application running with different parameters has different functionalitis
///
/// 1. None parameter:
/// 2. c: run the camera calibration mode
/// 3. t: run the test mode
/// 4~...: run with other custum mode*
/// finally. DEFAULT: run the fight mode
/// \Case Our Sentry will run with different conditions
/// Scout:1) move fast and camera
/// StaticAttack:
/// DynamicAttack:
/// Retreat:*
inline std::string getTimePictureC() {
    time_t TimeValue;
    time (&TimeValue);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "../others/picture/Calibration/group1/%Y-%m-%d_%H_%M_%S.png",localtime(&TimeValue));
    return tmp;
}
int main(int argv,char **argc) {
    ALLIANCE_PRINT("Hello Aliance!!");
    ALLIANCE_PRINT2("The Current path",argc[0]);

    Setter setter;
    if(argv == 1){
        RoboMaster2019 roboMaster2019(setter);

        roboMaster2019.SentryMode();
    }else if(argv == 2 && argc[1][0] == 'c'){         //Run in a special mode -- Clibration functionality
        Calibrator calibrator(setter.Calibration.data(),setter.CornersMatrix,setter.Rectangle);
        calibrator.save_calibrate_result();
    }else if(argv == 2 && argc[1][0] == 't'){         //Run in a special mode -- Test
        std::cout << "Alliance Nanjing University of Science & Technology Nanjing Jiangsu China" << std::endl;
    }else if(argv == 2 && argc[1][0] == 'n'){

    }else{
        std::cout << "Please Select one correct parameter!" << std::endl;
    }
    return 0;
}