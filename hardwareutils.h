#ifndef __HARDWAREUTILS_H__
#define __HARDWAREUTILS_H__

#define TRUE 1
#define FALSE 0


#define LED_TOGGLE(x) digitalWrite(x, digitalRead(x) ? LOW : HIGH)
#define LED_ON(x) digitalWrite(x, HIGH)
#define LED_OFF(x) digitalWrite(x, LOW)


#endif
