#include <Servo.h>

Servo myServo;
int current_position = 90; // Start at neutral position

void setup()
{
    Serial.begin(9600);       // Start serial communication
    myServo.attach(9);        // Attach the servo to pin 9
    myServo.write(current_position); // Set initial position
    
}

void loop()
{
    if (Serial.available() > 0)
    {
        int new_position = Serial.parseInt(); // Read the new position from serial input
        if (new_position > 0 && new_position <= 180)
        {
            current_position = new_position;  // Update servo position
            myServo.write(current_position);  // Move the servo
        }
    }
}
