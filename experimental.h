

/*
void setup() {
    declarations();

    //for the LED requirements
    elapsedMillis T_ledTrack;
    bool led_status = 1;

    //the preflight loop, runs until liftoff
    elapsedMillis T_preLiftoff;
    elapsedMillis T_sinceLiftoff;
    while(!status.liftoff) {
        bmp_get_data();
        imu_get_data();
        //Serial.println("liftoff");
        data_log(T_preLiftoff, T_sinceStart);
        status_liftoff(T_sinceLiftoff);
        if(T_ledTrack >= 200) {
            led_status = !led_status;
            T_ledTrack = 0;
        }
    }

    //launch has occured, runs until burnout
    elapsedMillis T_sinceBurnout;
    while(!status.burnout) {
        bmp_get_data();
        imu_get_data();
        //Serial.println("burnout");
        data_log(T_sinceLiftoff, T_sinceStart);
        status_burnout(T_sinceBurnout);
    }

    //wait for two seconds before moving on to the main control loop(contest rules)
    elapsedMillis T_waiting;
    while(T_waiting < wait_time) {
        bmp_get_data();
        imu_get_data();
        //Serial.println("waiting");
        data_log(T_sinceBurnout, T_sinceStart);
    }

    //do these to setup for control functions after the burnout wait period
    T_sinceControl = 0;
    T_sincePulsed = Control_period;
    T_sinceHold = 0;
    T_lastTime = 0;
    T_sinceApogee = 0;
    T_sinceFalling = 0;
    T_sinceLoop = 0;
    current_command = 0;
}


void loop() {
    bmp_get_data();
    imu_get_data();
    data_log(T_sinceControl, T_sinceStart);

    control();
    valveWrite();

    checkApogee();
}

void checkApogee() {
    //CHANGE THE NUMBER FOR APOGEE TIMING//
    if(T_sinceLoop >= 20000) {
        elapsedMillis T_ledTrack;
        bool led_status = 1;
        while(1) {
            bmp_get_data();
            imu_get_data();
            data_log(T_sinceApogee, T_sinceStart);

            if(T_ledTrack >= 500) {
                led_status = !led_status;
            }

            if(dumpTime >= 12000) {
                digitalWrite(valve_dump, LOW);
            }
        }
    } else {
        T_sinceApogee = 0;
    }
}

void data_log(elapsedMillis current_time, elapsedMillis global_time) {
    if(T_sinceLogged >= log_period_T) {

        sd_write(current_time, global_time);
        //print_stuff(current_time, global_time);

        T_sinceLogged = 0;
    }
}

    pinMode(SD_chipSelect, OUTPUT);
    if(!SD.begin(SD_chipSelect)) {
        Serial.println("initialization failed");
        delay(500);
    }
    sd_setup();
*/



/*
static elapsedMillis T_sinceApogee;
static elapsedMillis T_sinceFalling;

static elapsedMillis T_sinceLogged;
static elapsedMillis T_statuscheck;
static elapsedMillis T_sinceStart;
static elapsedMillis T_sinceEmergency;


struct status_set {
    bool liftoff = 0;
    bool burnout = 0;
    bool falling = 0;
    bool emergency = 0;
    bool active_valves = 0;
    bool IMU_connected = 0;
    bool BMP_connected = 0;
};
extern struct status_set status; //the struct holding the status of all systems


#define lift_check_T 50
#define burn_check_T 50
#define apo_check_T 50
#define fall_check_T 50
#define liftoff_Gforce_check 6
#define burnout_Gforce_check 3
//#define apogee_velocity_check 2.5 //check should be about 2.5 m/s


//perform status check and update flags, will disbale control and trigger landing loop code when apogee has been reached, will activate emergency dump valve if given signal by ground, etc.
void status_liftoff(elapsedMillis T_sinceLiftoff);
void status_burnout(elapsedMillis T_sinceBurnout);
void status_falling(elapsedMillis T_sinceFalling);


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
}

void status_falling(elapsedMillis T_sinceFalling) {
    if(bmp.altitude < (bmp.max_altitude-100)) {
        T_sinceFalling = 0;
    } else {
        if (T_sinceFalling > fall_check_T) {
            status.falling = true;
        }
    }
}

*/




// #ifndef H_h
// #define H_h
 // struct globe
  // {
    // float x;
    // float y;
    // float z;
    // float accx;
    // float accy;
    // float accz;
  // };
// void SPI_SETUP();
// void Accelerometer_Setup();
// void accel();
// void readVal();



/*

//#include "SparkFunLIS3DH.h"
#include "Wire.h"
#include "SPI.h"
#include "H.h"

LIS3DH myIMU; //Default constructor is I2C, addr 0x19.
void setup() {
Serial.begin(9600);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");

  //Call .begin() to configure the IMU
  myIMU.begin();
}

void loop() {
   #include "H.h"
 
//*************************************************************************************************************
//Writes gyro to serial print  
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatAccelX(), 4);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatAccelY(), 4);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatAccelZ(), 4);

  delay(500);
//**************************************************************************************************************   
void SPI_SETUP();
void Accelerometer_Setup();
void accel();
void readVal();

globe AC;
Serial.print("Accel X = ");
  Serial.print(AC.accx);
  Serial.print(" Accel Y = ");
  Serial.print(AC.accy);
  Serial.print(" Accel Z = ");
  Serial.println(AC.accz);
delay(100);
}


*/


/*
#include <Adafruit_Sensor.h>
#include <Wire.h>

const long interval = 1000;   // interval at which to call the function (milliseconds)

unsigned long previousMillis = 0;



void loop() {

        unsigned long currentMillis = millis();
        int Time = currentMillis * .001;


        if (currentMillis - previousMillis >= interval) {
                previousMillis = currentMillis;
                Serial.print(F("Time = ")); Serial.print(Time);
                Serial.println(F(""));

                dataFile = SD.open(F("data.txt"),FILE_WRITE);
                dataFile.print(Time);
                dataFile.print(F(","));
                dataFile.close();
        }

}

// void gpsData() {

        // float Latitude = tinyGPS.location.lat(), Longitude = tinyGPS.location.lng();
        // double gpsAltitude = tinyGPS.altitude.feet(), Course = tinyGPS.course.deg(), Speed = tinyGPS.speed.mph();
        // int Satellites = tinyGPS.satellites.value();

        // Serial.print(F("Lat: ")); Serial.println(Latitude, 6);
        // Serial.print(F("Long: ")); Serial.println(Longitude, 6);
        // Serial.print(F("Alt: ")); Serial.println(gpsAltitude);
        // Serial.print(F("Course: ")); Serial.println(Course);
        // Serial.print(F("Speed: ")); Serial.println(Speed);
        // Serial.print(F("Satillites: ")); Serial.println(Satellites);
        // Serial.println();

        // dataFile = SD.open(F("data.txt"),FILE_WRITE);
        // dataFile.print(Latitude, 6);
        // dataFile.print(F(","));
        // dataFile.print(Longitude, 6);
        // dataFile.print(F(","));
        // dataFile.print(gpsAltitude);
        // dataFile.print(F(","));
        // dataFile.print(Course);
        // dataFile.print(F(","));
        // dataFile.print(Speed);
        // dataFile.print(F(","));
        // dataFile.print(Satellites);
        // dataFile.println(F(""));
        // dataFile.close();

// }

/////////////////////////////////////////////////////////////////////////////////////////////
// struct bmp_data bmp;


// #include <Status.h>
// struct status_set status; //the struct holding the status of all systems

// void setup() {
    // elapsedMillis T_preLiftoff;
    // elapsedMillis T_sinceLiftoff;
    // while(!status.liftoff) {
        // bmp_get_data();
        // imu_get_data();
        // data_log(T_preLiftoff, T_sinceStart);
        // status_liftoff(T_sinceLiftoff);
        // if(T_ledTrack >= 200) {
            // led_status = !led_status;
            // T_ledTrack = 0;
        // }
    // }

    // elapsedMillis T_sinceBurnout;
    // while(!status.burnout) {
        // bmp_get_data();
        // imu_get_data();
        // data_log(T_sinceLiftoff, T_sinceStart);
        // status_burnout(T_sinceBurnout);
    // }

    // elapsedMillis T_waiting;
    // while(T_waiting < wait_time) {
        // bmp_get_data();
        // imu_get_data();
        // data_log(T_sinceBurnout, T_sinceStart);
    // }

    // T_sinceControl = 0;
    // T_sincePulsed = Control_period;
    // T_sinceHold = 0;
    // T_lastTime = 0;
    // T_sinceApogee = 0;
    // T_sinceFalling = 0;
    // T_sinceLoop = 0;
    // current_command = 0;
// }


// void loop() {
    // bmp_get_data();
    // imu_get_data();
    // data_log(T_sinceControl, T_sinceStart);

    // control();
    // valveWrite();

    // checkApogee();

    // /*
    // T_sinceEmergency = 0;
    // while(status.emergency) {
        // bmp_get_data();
        // imu_get_data();
        // digitalWrite(valve_dump, HIGH);
        // data_log(T_sinceEmergency, T_sinceStart);
    // }
    // */
// }


// void checkApogee() {
    // if(T_sinceLoop >= 20000) {
        // elapsedMillis T_ledTrack;
        // bool led_status = 1;
        // while(1) {
            // bmp_get_data();
            // imu_get_data();
            // data_log(T_sinceApogee, T_sinceStart);

            // if(T_ledTrack >= 500) {
                // led_status = !led_status;
            // }
        // }
    // } else {
        // T_sinceApogee = 0;
    // }
// }

// void data_log(elapsedMillis current_time, elapsedMillis global_time) {
    // if(T_sinceLogged >= log_period_T) {

        // sd_write(current_time, global_time);

        // T_sinceLogged = 0;
    // }
// }

// void declarations() {
    // pinMode(SD_chipSelect, OUTPUT);
    // if(!SD.begin(SD_chipSelect)) {
        // Serial.println("initialization failed");
        // delay(500);
    // }
    // sd_setup();
// }
/////////////////////////////////////////////////////////////////////////////////////////////
// const int ChipSelectPin = BUILTIN_SDCARD;
// #define SD_chipSelect BUILTIN_SDCARD
// extern File myFile;


// void status_liftoff(elapsedMillis T_sinceLiftoff) {
    // if(((abs(imu_data.aSqrt)) <= liftoff_Gforce_check)) {
        // T_sinceLiftoff = 0;
        // imu_data.theta_roll = 0;
        // imu_data.omega_roll_last = 0;
    // } else {
        // if(T_sinceLiftoff >= lift_check_T)
            // status.liftoff = true;
    // }
// }

// void status_burnout(elapsedMillis T_sinceBurnout) {
    // if((abs(imu_data.aSqrt)) >= burnout_Gforce_check){
      // T_sinceBurnout = 0;
    // } else {
      // if(T_sinceBurnout >= burn_check_T)
        // status.burnout = true;
    // }
// }

// void status_falling(elapsedMillis T_sinceFalling) {
    // if(bmp.altitude < (bmp.max_altitude-100)) {
        // T_sinceFalling = 0;
    // } else {
        // if (T_sinceFalling > fall_check_T) {
            // status.falling = true;
        // }
    // }
// }

// #ifndef _STATUS_H_
// #define _STATUS_H_

// #include "Global.h"
// #include <IMU.h>
// #include <BMP085.h>


// struct status_set {
    // bool liftoff = 0;
    // bool burnout = 0;
    // bool falling = 0;
    // bool emergency = 0;
    // bool active_valves = 0;
    // bool IMU_connected = 0;
    // bool BMP_connected = 0;
// };
// extern struct status_set status; //the struct holding the status of all systems


// #define lift_check_T 50
// #define burn_check_T 50
// #define apo_check_T 50
// #define fall_check_T 50
// #define liftoff_Gforce_check 6
// #define burnout_Gforce_check 3

// void status_liftoff(elapsedMillis T_sinceLiftoff);
// void status_burnout(elapsedMillis T_sinceBurnout);
// void status_falling(elapsedMillis T_sinceFalling);
// void flight_plan();


// /*    //GROUP 1 DATA
    // float setpoint = 0.0;
    // float last_setpoint = 0.0;
    // int hold_time = 0;
    // int global_time = 0; //the synced timing
    // int local_time = 0; //timing of the current system state

    // double pressure;
    // double temperature;
    // double altitude;
    // double max_altitude;
// */
// /*    //GROUP 2 DATA
    // float aX;
    // float aY;
    // float aZ;
    // float aSqrt;
    // float gX;
    // float gY;
    // float gZ;
    // float mDirection;
    // float mX;
    // float mY;
    // float mZ;
// */
// /*  //GROUP 3 DATA
    // int current_command;

    // float error;
    // float d_error;
    // float d_error_sum;
    // float d_error_cnt;

    // float omega_roll;
    // float theta_roll;

    // bool liftoff = 0;
    // bool burnout = 0;
    // bool falling = 0;
    // bool emergency = 0;
    // bool is_done = 0;
    // bool roll_direction = 0;
    // bool active_valves = 0;

    // bool IMU_connected = 0;
    // bool BMP_connected = 0;
// */





/*************************************

*************************************/

/*
void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}
*/


/* 
void CheckLEDs(void)
{
  if (millis() >= NextLEDs)
  {
    static byte Flash=0;
    
    // This would normally be the only LED for status (i.e. no OK or WARN LEDs)
    if (GPS.Altitude > 1000)
    {
      // All off
      ControlLEDs(0,0,0);
    }
    else if ((GPS.FixType == 3) && (GPS.Satellites >= 4))
    {
      ControlLEDs(Flash, Flash, 0);
    }
    else
    {
      ControlLEDs(1, 0, Flash);
    }       
    
    NextLEDs = millis() + 500L;
    Flash = 1-Flash;
  }
}
*/



    /*if (APRS_SLOT >= 0) {
        //transmission_timer = millis() //+ 1000 * (APRS_PERIOD - (gps_seconds + APRS_PERIOD - APRS_SLOT) % APRS_PERIOD);
        transmission_timer = millis() + (1000L * APRS_SLOT);
    } else {
        transmission_timer = millis();
    }*/