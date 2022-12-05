#ifndef _STATUS_H_
#define _STATUS_H_


#include "BMP.h"
#include "GPS.h"
#include "accel.h"
#include "SD_card.h"

#include "config.h"
#include "debugutils.h"
#include "hardwareutils.h"

#define lift_check_T 50
#define burn_check_T 50
#define apo_check_T 50
#define fall_check_T 50
#define liftoff_Gforce_check 6
#define burnout_Gforce_check 3
//#define apogee_velocity_check 2.5 //check should be about 2.5 m/s



class STATUS {
    private:
        bool liftoff = 0;
        bool burnout = 0;
        bool falling = 0;
        //bool emergency = 0;
        
        bool ValidBMP = 0;
        bool ValidSD = 0;
        
        void CheckSensors();
        void CheckLiftoff();
        void CheckBurnout();
        void CheckFalling();

    public:
        STATUS();
        ~STATUS() {};
        
        void CheckStatus();
        
        bool IsLiftoff();
        bool IsBurnout();
        bool IsFalling();
        bool IsValidBMP();
        bool IsValidSD();
};

extern STATUS status;

#endif
