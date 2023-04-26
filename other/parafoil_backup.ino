
const int JackPin = 2;
const int ledPin = 13;
unsigned long StartTime;
unsigned long StopTime;
unsigned long duration;
#include <Servo.h>

Servo servo1;
Servo servo2;
bool doMove = false;

void setup()
{

    pinMode(ledPin, OUTPUT);
    pinMode(JackPin, INPUT);
    digitalWrite(JackPin, HIGH);
    servo1.attach(9);
    servo2.attach(8);
    Serial.begin(9600);
    servo1.write(0);
    servo2.write(0);
}

void loop()
{
    if (digitalRead(JackPin) == HIGH)
    {
        StartTime = millis();
        digitalWrite(ledPin, LOW);
    }

    else
    {
        digitalWrite(ledPin, HIGH);
        StopTime = millis();
        duration = StopTime - StartTime;
        double durationSec = (double)duration / 1000.0;
        Serial.print("Duration in Seconds: ");
        Serial.println(durationSec);
        if (durationSec >= 10)
        {
            Serial.println();
            for (int i = 0; i <= 180; i++)
            {
                servo1.write(i);
                servo2.write(i);
                delay(3);
            }
        }
    }
}