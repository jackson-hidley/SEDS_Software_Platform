#include "Status.h"


STATUS::STATUS() {
    
}


void STATUS::CheckSensors() {
    
    #ifdef BMP_ENABLE
        ValidBMP = bmp.IsValidBMP();
        if(!ValidBMP) {
            DEBUG_PRINT(F("Could not find a valid BMP085 sensor."));
            LED_ON(RED_LED);
        }
    #endif

    #ifdef SD_ENABLE
        ValidSD = sd.IsValidSD();
        if(!ValidSD) {
            DEBUG_PRINT(F("SD card problem."));
            LED_ON(RED_LED);
        }
    #endif   
}


void STATUS::CheckLiftoff() {
    liftoff = 1;
}


void STATUS::CheckBurnout() {
    burnout = 1;
}


void STATUS::CheckFalling() {
    if( bmp.GetAltitude() < (bmp.GetMaxAltitude() - 100) ) {
        falling = 1;
    }
}
    

void STATUS::CheckStatus() {
    this->CheckSensors();
    
    if(!liftoff) {
        this->CheckLiftoff();
    } else if(!burnout) {
        this->CheckBurnout();
    } else if(!falling) {
        this->CheckFalling();
    }
}
    
/*
void status_liftoff(elapsedMillis T_sinceLiftoff) {
    if(((abs(imu_data.aSqrt)) <= liftoff_Gforce_check)) {
        T_sinceLiftoff = 0;
        imu_data.theta_roll = 0;
        imu_data.omega_roll_last = 0;
        //data.setNorth();
    } else {
        if(T_sinceLiftoff >= lift_check_T)
            status.liftoff = true;
    }
}

void status_burnout(elapsedMillis T_sinceBurnout) {
    if((abs(imu_data.aSqrt)) >= burnout_Gforce_check){
      T_sinceBurnout = 0;
    } else {
      if(T_sinceBurnout >= burn_check_T)
        status.burnout = true;
    }
}*/


bool STATUS::IsLiftoff() {
    return liftoff;
}

bool STATUS::IsBurnout() {
    return burnout;
}

bool STATUS::IsFalling() {
    return falling;
}

bool STATUS::IsValidBMP() {
    return ValidBMP;
}

bool STATUS::IsValidSD() {
    return ValidSD;
}
