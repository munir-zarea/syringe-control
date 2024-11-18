#include <Servo.h>

Servo myServo;          // Create a Servo object to control the servo motor
int servoPos = 90;      // Start at a neutral position (90 degrees)

void setup() {
    Serial.begin(9600);   // Set the baud rate to match the ROS2 node
    myServo.attach(9);    // Attach the servo to digital pin 9
    myServo.write(servoPos);  // Move the servo to the starting position
}

void loop() {
    // Check if there's incoming data on the serial port
    if (Serial.available() > 0) {
        int receivedPos = Serial.parseInt();  // Read an integer from the serial input

        // Validate that the position is within the servo's range (0 to 180)
        if (receivedPos >= 0 && receivedPos <= 180) {
            servoPos = receivedPos;           // Update the servo position
            myServo.write(servoPos);          // Move the servo to the new position

            // Optional: Print confirmation to Serial Monitor for debugging
            Serial.print("Servo moved to: ");
            Serial.println(servoPos);
        }
    }
}
