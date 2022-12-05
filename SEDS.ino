#include <SoftwareSerial.h>
#include "TinyGPS++.h"

#include "config.h"
#include "debugutils.h"
#include "hardwareutils.h"

#ifdef GPS_ENABLE
    #include "GPS.h"
    GPS gps;
#endif

#ifdef BMP_ENABLE
    #include "BMP.h"
    BMP bmp;
#endif

#ifdef ACCEL_ENABLE
    #include "accel.h"
    ACCEL accel;
#endif

#ifdef RTTY_ENABLE
    #include "RTTY.h"
    RTTY rtty;
#endif

#ifdef APRS_ENABLE
    #include "APRS.h"
#endif

#ifdef SD_ENABLE
    #include "SD_card.h"
    SD_card sd;
#endif

#ifdef STATUS_ENABLE
    #include "status.h"
    STATUS status;
#endif



void setup()
{
    DEBUG_UART.begin(230400);
    //pinMode(RED_LED, OUTPUT);
    //pinMode(GREEN_LED, OUTPUT);

    //LED_ON(RED_LED);
    //LED_ON(GREEN_LED);
    //delay(1000);
    //LED_OFF(RED_LED);
    //LED_OFF(GREEN_LED);
    
    #ifdef GPS_ENABLE
        gps.GPS_Setup();
        delay(75);
    #endif
    
    #ifdef ACCEL_ENABLE
        //accel.ACCEL_Setup();
        //delay(75);
    #endif
    
    #ifdef BMP_ENABLE 
        bmp.BMP_Setup();
        delay(75);
    #endif

    #ifdef RTTY_ENABLE
        rtty.RTTY_Setup();
        delay(75);
        rtty.RTTY_SetChannel();
        delay(75);
    #endif
    
    #ifdef APRS_ENABLE
        APRS_Setup( 50,      // number of preamble flags to send
                    PTT_PIN, // Use PTT pin
                    100,     // ms to wait after PTT to transmit
                    0, 0     // No VOX tone
                    );
        delay(75);
    #endif
    
    #ifdef SD_ENABLE
        sd.SD_Setup();
        delay(75);
    #endif
    
    #ifdef STATUS_ENABLE
        status.CheckStatus();
    #endif
    
    DEBUG_PRINT(F("Main Setup successful"));
}


void loop() {  
    #ifdef GPS_ENABLE
        gps.GPS_Update(); 
    #endif
        
    #ifdef ACCEL_ENABLE
        accel.ACCEL_Update();
    #endif
    
    #ifdef BMP_ENABLE 
        bmp.BMP_Update();
    #endif
    
    #ifdef RTTY_ENABLE
        //LED_ON(GREEN_LED);
        rtty.check_RTTY();
        //LED_OFF(GREEN_LED);
    #endif
    
    #ifdef APRS_ENABLE
        //LED_ON(GREEN_LED);
        check_APRS();
        //LED_OFF(GREEN_LED);
    #endif
    
    #ifdef STATUS_ENABLE
        status.CheckStatus();
    #endif

    #ifdef SD_ENABLE
        sd.SD_Record();
        delay(1);
    #endif
}
