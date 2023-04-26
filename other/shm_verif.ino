void setup()
{
    Serial.begin(9600);
}

void loop()
{
    int sensorValue = analogRead(A4);

    Serial.print("Analog value: ");
    Serial.println(float(sensorValue) / 100 - 1.00);

    delay(1000);
}