#include "RTTY.h"
#include "GPS.h"

static unsigned long rtty_timer = 0;

RTTY::RTTY() : NTX2EN(13, RADIOEN){
    SoftwareSerial NTX2EN(13, RADIOEN);
    //delay(10);
    //this->RTTY_SetChannel();
}


void RTTY::check_RTTY() {
    static unsigned long MessageDelay = 0;  //takes account of delayed APRS messages due to GPS signal loss
    
    // Time for another APRS frame
    if ((int32_t) (millis() - rtty_timer) >= 0) {
        if (gps.ValidLocation() && (gps.AgeLocation() < (5 * GPS_VALID_POS_TIMEOUT) ) ) {
            RTTY::Transmit_RTTY_GPS();
            rtty_timer += ( (RTTY_PERIOD * 1000L) + MessageDelay);
            MessageDelay = 0;
            DEBUG_PRINT("\nRTTY with GPS sent.\n");
        } /*else {
            RTTY::Transmit_RTTY_NO_GPS();
            rtty_timer += ( (RTTY_PERIOD * 1000L) + MessageDelay);
            MessageDelay = 0;
            RTTY_DEBUG(F("\nRTTY without GPS sent.\n"));
        }*/
    } else {
        MessageDelay = millis();
        DEBUG_PRINT("\nrtty notsend\n");
    }
}


void RTTY::RTTY_Setup() {
    pinMode(RTTY_DATA_PIN, OUTPUT);
     
}


void RTTY::RTTY_SetChannel() {
     // RX, TX
     int i=0;
    for (i = 1; i <= 2; i++) 
    { 
    SoftwareSerial NTX2EN(13, RADIOEN);   
    NTX2EN.begin(RTTY_BAUDRATE);
      delay(100); 
        NTX2EN.write(0x80);
          NTX2EN.print("wr");
          NTX2EN.print(RTTY_Channel);
          NTX2EN.print("\r");
      delay(100);
    NTX2EN.end();      
    }
  
}


void RTTY::rtty_txbit (int bit) {
    /* NOTE: This new analog write values are for 12 bit analogwriteresolution.
    **       See afsk setup for details. Values for 8 bit default resolution are
    **       110 high and 100 low
    */
    if (bit) {
        // high
        analogWrite(RTTY_DATA_PIN, 1766);
    }
    else {
        // low
        analogWrite(RTTY_DATA_PIN, 1606);
    }

     delayMicroseconds(3370); // 300 baud
    //delayMicroseconds(10000); // For 50 Baud uncomment this and the line below.
    //delayMicroseconds(10150);
  
}


void RTTY::rtty_txbyte (char c) {
    /* Simple function to sent each bit of a char to
    ** rtty_txbit function.
    ** NB The bits are sent Least Significant Bit first
    **
    ** All chars should be preceded with a 0 and
    ** proceded with a 1. 0 = Start bit; 1 = Stop bit
    **
    */

    int i;

    rtty_txbit (0); // Start bit

    // Send bits for for char LSB first

    for (i = 0; i < 7; i++) { // Change this here 7 or 8 for ASCII-7 / ASCII-8
        if (c & 1) {
            rtty_txbit(1);
        }
        else {
            rtty_txbit(0);
        }
        
        c = c >> 1;
    }

    rtty_txbit (1); // Stop bit
    rtty_txbit (1); // Stop bit
    
}


void RTTY::rtty_txstring (char *string) {
    /* Simple function to sent a char at a time to
    ** rtty_txbyte function.
    ** NB Each char is one byte (8 Bits)
    */

    char c;

    c = *string++;

    while ( c != '\0') {
        rtty_txbyte (c);
        c = *string++;
    }
}


void RTTY::Transmit_RTTY_GPS() {
    digitalWrite(RADIOEN, HIGH);
    snprintf(datastring, 22, "%f  %f",gps.GetLatitude(),gps.GetLongitude()); // Puts the text in the datastring
    strcat(datastring, "\n");
    rtty_txstring (datastring);
}

void RTTY::Transmit_RTTY_NO_GPS() {
    digitalWrite(RADIOEN, HIGH);
    snprintf(datastring, 22, "KN4JLK RTTY NO GPS"); // Puts the text in the datastring
    strcat(datastring, "\n");
    rtty_txstring (datastring);
}
