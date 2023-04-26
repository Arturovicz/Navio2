import serial
import math
import navio.util
import navio.ublox
import navio.pwm
import RPi.GPIO as GPIO

import sys
import time
import navio.ms5611

navio.util.check_apm()
baro = navio.ms5611.MS5611()
baro.initialize()

ser = open("/home/pi/data.txt", "a")
ser.write(str("nouveau "))
# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# LED indicaring that the system is on
LED_POWER = 24
GPIO.setup(LED_POWER, GPIO.OUT)
GPIO.output(LED_POWER, GPIO.HIGH)

# pin for retrieving the information of th removed jack
jack = 25
GPIO.setup(jack, GPIO.IN)

# read the jack status
status = GPIO.input(jack)

# Def pins motors
M1_En = 13
M1_In1 = 6
M1_In2 = 5
M2_En = 18
M2_In1 = 17
M2_In2 = 27

# Creation of a pin list for each engine to compact a part of the code
Pins = [[M1_En, M1_In1, M1_In2], [M2_En, M2_In1, M2_In2]]

# setup of the pins motors driver bridge
GPIO.setup(M1_En, GPIO.OUT)
GPIO.setup(M1_In1, GPIO.OUT)
GPIO.setup(M1_In2, GPIO.OUT)

GPIO.setup(M2_En, GPIO.OUT)
GPIO.setup(M2_In1, GPIO.OUT)
GPIO.setup(M2_In2, GPIO.OUT)

M1_Vitesse = GPIO.PWM(M1_En, 200)
M2_Vitesse = GPIO.PWM(M2_En, 200)


# Start motors function
def marcheVitesse(num):
    if num == 1:
        M1_Vitesse.start(100)
    else:
        M2_Vitesse.start(100)


# Stop speed motors function
def arretVitesse(num):
    if num == 1:
        M1_Vitesse.stop()
    else:
        M2_Vitesse.stop()


# Operation motor in direction 1
def sens1(moteurNum):
    marcheVitesse(moteurNum)
    GPIO.output(Pins[moteurNum - 1][1], GPIO.HIGH)
    GPIO.output(Pins[moteurNum - 1][2], GPIO.LOW)
    print("Moteur", moteurNum, "tourne dans le sens 1.")


# Operation motor in direction 2
def sens2(moteurNum):
    marcheVitesse(moteurNum)
    GPIO.output(Pins[moteurNum - 1][1], GPIO.LOW)
    GPIO.output(Pins[moteurNum - 1][2], GPIO.HIGH)
    print("Moteur", moteurNum, "tourne dans le sens 2.")


# Stop motors function
def arret(moteurNum):
    GPIO.output(Pins[moteurNum - 1][1], GPIO.LOW)
    GPIO.output(Pins[moteurNum - 1][2], GPIO.LOW)
    print("Moteur", moteurNum, "arret.")


# Stop all the motors
def arretComplet():
    GPIO.output(Pins[0][1], GPIO.LOW)
    GPIO.output(Pins[0][2], GPIO.LOW)
    print("Moteurs arretes.")


arretComplet()


# Servo operation function
def servo(sens, numServo, temps):
    if sens == 1:
        sens1(numServo)
        time.sleep(temps)
        arret(numServo)
    else:
        sens2(numServo)
        time.sleep(temps)

    return


# create a vector
def vecteur(xA, yA, xB, yB):
    return [xB - xA, yB - yA]


# read the prussure
preS = [0, 0, 0]
baro.refreshPressure()
time.sleep(0.01)  # Waiting for pressure data ready 10ms
baro.readPressure()
baro.refreshTemperature()
time.sleep(0.01)  # Waiting for temperature data ready 10ms
baro.readTemperature()
baro.calculatePressureAndTemperature()
preS[0] = baro.PRES

baro.refreshPressure()
time.sleep(0.01)  # Waiting for pressure data ready 10ms
baro.readPressure()
baro.refreshTemperature()
time.sleep(0.01)  # Waiting for temperature data ready 10ms
baro.readTemperature()
baro.calculatePressureAndTemperature()
preS[1] = baro.PRES

baro.refreshPressure()
time.sleep(0.01)  # Waiting for pressure data ready 10ms
baro.readPressure()
baro.refreshTemperature()
time.sleep(0.01)  # Waiting for temperature data ready 10ms
baro.readTemperature()
baro.calculatePressureAndTemperature()
preS[2] = baro.PRES

out1 = False
while not out1:
    if preS[1] >= preS[0] >= preS[2]:
        out1 = True
    elif preS[2] >= preS[0] >= preS[1]:
        out1 = True
    else:
        att = preS[0]
        preS[0] = preS[1]
        preS[1] = preS[2]
        preS[2] = att

pressionSol = preS[0]
ser.write(str("pressionSol :\n"))
ser.write(str(pressionSol))


# calculation of the altitude with the pressure
def altitudeFus():
    preS = [0, 0, 0]
    baro.refreshPressure()
    time.sleep(0.01)  # Waiting for pressure data ready 10ms
    baro.readPressure()
    baro.refreshTemperature()
    time.sleep(0.01)  # Waiting for temperature data ready 10ms
    baro.readTemperature()
    baro.calculatePressureAndTemperature()
    preS[0] = baro.PRES

    baro.refreshPressure()
    time.sleep(0.01)  # Waiting for pressure data ready 10ms
    baro.readPressure()
    baro.refreshTemperature()
    time.sleep(0.01)  # Waiting for temperature data ready 10ms
    baro.readTemperature()
    baro.calculatePressureAndTemperature()
    preS[1] = baro.PRES

    baro.refreshPressure()
    time.sleep(0.01)  # Waiting for pressure data ready 10ms
    baro.readPressure()
    baro.refreshTemperature()
    time.sleep(0.01)  # Waiting for temperature data ready 10ms
    baro.readTemperature()
    baro.calculatePressureAndTemperature()
    preS[2] = baro.PRES

    out1 = False
    while not out1:
        if preS[1] >= preS[0] >= preS[2]:
            out1 = True
        elif preS[2] >= preS[0] >= preS[1]:
            out1 = True
        else:
            att = preS[0]
            preS[0] = preS[1]
            preS[1] = preS[2]
            preS[2] = att
    pres = preS[0]
    alt = math.log(pres / pressionSol) * (-2 * 1006 * 20) / (7 * 9.81)
    return alt


def vitAltitude(alt1, alt2, t):
    vz = math.fabs(alt2 - alt1) / t
    return vz


# reference altitude
def altitudeArea():
    return 430


# check the altitude of the rocket
def isAltitudeZero():
    if altitudeFus() <= 30:
        return True
    else:
        return False


# servo control to land the rocket
def safeLanding():
    servo(1, 1, 1.5)
    while not isAltitudeZero():
        GPIO.output(LED_POWER, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(LED_POWER, GPIO.LOW)
        time.sleep(0.5)
        servo(1, 1, 0.25)
    return


# servo control to land the rocket
def landing():
    servo(1, 1, 1.5)
    while not isAltitudeZero():
        GPIO.output(LED_POWER, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(LED_POWER, GPIO.LOW)
        time.sleep(0.5)
        servo(1, 1, 0.25)
    return


# main navigation function
def droite(temps):
    pinR = 2  # pin servo right
    #pinL = 1  # pin servo left
    servo(1, pinR, temps)
    time.sleep(2)


def gauche(temps):
    #pinR = 2  # pin servo right
    pinL = 1  # pin servo left
    servo(1, pinL, temps)
    time.sleep(2)


def main():
    count = 0
    out = False
    var = False
    # infinite loop
    while not out:

        # if the jack is unpluged
        if GPIO.input(jack) == 1 or var == True:
            print("prisejackdebranchée\n")
            if not var:
                # flashes the LED to verify that the program is running
                ser.write(str("prise jack debranchee\n"))
                GPIO.output(LED_POWER, GPIO.LOW)
                time.sleep(1)
                GPIO.output(LED_POWER, GPIO.HIGH)
                time.sleep(1)
                GPIO.output(LED_POWER, GPIO.LOW)
                time.sleep(1)
                GPIO.output(LED_POWER, GPIO.HIGH)

                var = True

                # deployment timing of the parafoil
                time.sleep(15)

            alt1 = altitudeFus()
            ser.write("alt1")
            ser.write(str(alt1))
            time.sleep(1)
            alt2 = altitudeFus()
            ser.write("alt2")
            ser.write(str(alt2))

            if vitAltitude(alt1, alt2, 1.12) < 5 or vitAltitude(alt1, alt2, 1.12) > 15 or alt1 < 30 or alt2 < 30:
                safeLanding()
                ser.write(str("vitesse altitude trop faible ou trop elevee : atterrissage \n"))
                ser.write(str(vitAltitude(alt1, alt2, 1.12)))
                return

            # phase 1 experiment
            elif count == 0:
                count = count + 1
                time.sleep(3)
                ser.write(str("1ere stabilisation\n"))

            # phase 2 experiment
            elif count == 1:
                droite(0.75)
                count = count + 1
                ser.write(str("action servo droite 0.75\n"))

            # phase 3 experiment
            elif count == 2:
                droite(1.5)
                count = count + 1
                ser.write(str("action servo droite 1.5\n"))

            # phase 4 experiment
            elif count == 3:
                # ne rien faire
                count = count + 1
                time.sleep(3)
                ser.write(str("2eme stabilisation\n"))

            # phase 5 experiment
            elif count == 4:
                gauche(0.75)
                count = count + 1
                ser.write(str("action servo gauche 0.75\n"))
            # phase 6 experiment
            elif count == 5:
                gauche(1.5)
                count = count + 1
                ser.write(str("action servo gauche 1.5\n"))

            # phase 7 experiment
            elif count == 6:
                # ne pas agir sur le parafoil
                count = count + 1
                time.sleep(3)
                ser.write(str("3eme stabilisation\n"))
            #phase 8 experiement
            elif count == 7:
                landing()
            ser.write(str("landing\n"))
            return
    return


if __name__ == "__main__":
    main()