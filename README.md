# Lab 6
## Moving with Purpose: Sensing and Mobility 
Aashika Uppala, Megan Fister

3/12/25

### Introduction
In this lab, we explore the fundamental principles of sensing and mobility for basic robotic applications. 
Specifically, we focus on using the HC-SR04 ultrasonic sensor to measure distances and integrate this data into a robotic system to enhance mobility and collision avoidance.
The lab introduces the concept of analog and digital sensors, explaining how they differ in terms of signal processing and application. 
Additionally, we work with actuators and an H-Bridge motor driver to control the movement of a two-motor system. 
By programming the Arduino to process sensor data and regulate motor behavior, we aim to create a simple autonomous collision-avoidance system. 
This lab provides hands-on experience with key components of robotic navigation and reinforces the importance of sensor integration in automated systems.
### Methods/Results
#### Instruments
•	A Computer running Arduino IDE

•	SparkFun Inventor’s kit

&nbsp; &nbsp; &nbsp; &nbsp; o	RedBoard

&nbsp; &nbsp; &nbsp; &nbsp; o	Ultrasonic sensor

&nbsp; &nbsp; &nbsp; &nbsp; o	Two motors

&nbsp; &nbsp; &nbsp; &nbsp; o	Motor Driver

#### Part 1 
First, we connected the ultrasonic sensor as shown below in Figure 1.

![Schematic 1](https://github.com/meganfister/Lab6/blob/main/Lab%206%20Schematic%201.png)

_Figure 1. Connection of the HC-SR04 Ultrasonic Sensor to the RedBoard (Schematic)_

_(https://learn.sparkfun.com/tutorial/sparkfun-inventors-kit-experiment-guide---v40/circuit-3b-distance-sensor)_

Next, we connected the RedBoard to the computer and start the Arduino IDE on the computer. Our constructed circuit is pictured below.

![Circuit 1](https://github.com/meganfister/Lab6/blob/main/Lab%206%20Calibration%20Distance.jpg)

_Figure 2. Connection of the HC-SR04 Ultrasonic Sensor to the RedBoard (Built Circuit)_

Using the code in https://projecthub.arduino.cc/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-7cabe1 for guidance, we programmed an algorithm to read the distance from the sensor and send the value over serial communications. The code we used is shown below. We changed the pin numbers to match the pins we used in our circuit.

```c++
const int trigPin = 10;
const int echoPin = 11;

float duration, distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);
}
```
Next, we used the ruler provided to test the algorithm.

To evaluate the resolution and precision of the ultrasonic sensor, we first calibrated the sensor using a known reference point. We positioned the function generator, which served as our measurement object, at 3.10 cm from the sensor, aligning it with the end of the black section on the board. We observed the sensor's output through the Arduino serial monitor. A photo of the function generator at calibration distance is pictured below.

![Calibration Distance](https://github.com/meganfister/Lab6/blob/main/Lab%206%20Calibration%20Distance%20Side%20View.jpg)

_Figure 3. Function generator at calibration distance_

To assess the sensor’s precision, we moved the function generator by 1 mm and measured 3.21 cm, pictured below.

![Moving object away from sensor](https://github.com/meganfister/Lab6/blob/main/Lab%206%20Moving%20Object%200.1.jpg)

_Figure 4. Measurement of function generator after it is moved 1 mm from calibration distance_

We then moved the function generator to 4 cm to test the sensor's precision at a slightly further distance, and the sensor measured 3.94 cm.


#### Part 2
Next, we moved the ultrasonic sensor connections on the RedBoard to pins 7 and 6. We connected the RedBoard, the Motor Driver, and the Motors as shown below in Figure 2. We did not include the sliding switch circuit in the diagram below.

![Schematic 2](https://github.com/meganfister/Lab6/blob/main/Lab%206%20Schematic%202.png)

_Figure 5. Two Motor Connection (Schematic)_

_(https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v40/circuit-5b-remote-controlled-robot)_

The built circuit can be seen in the picture below.

![Circuit 2](https://github.com/meganfister/Lab6/blob/main/Lab%206%20Circuit%20Part%202.jpg)

_Figure 6. Two Motor Connection (Built Circuit)_

Using https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v40/circuit-5b-remote-controlled-robot for guidance, we wrote a program to control the movement of both motors. The provided code is shown below.

```c++
/*
  SparkFun Inventor’s Kit
  Circuit 5B - Remote Control Robot

  Control a two wheeled robot by sending direction commands through the serial monitor.
  This sketch was adapted from one of the activities in the SparkFun Guide to Arduino.
  Check out the rest of the book at
  https://www.sparkfun.com/products/14326

  This sketch was written by SparkFun Electronics, with lots of help from the Arduino community.
  This code is completely free for any use.

  View circuit diagram and instructions at: https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v40
  Download drawings and code at: https://github.com/sparkfun/SIK-Guide-Code
*/


//the right motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 13;           //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;            //control pin 2 on the motor driver for the right motor
const int PWMA = 11;            //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;           //speed control pin on the motor driver for the left motor
const int BIN2 = 9;           //control pin 2 on the motor driver for the left motor
const int BIN1 = 8;           //control pin 1 on the motor driver for the left motor

int switchPin = 7;             //switch to turn the robot on and off

const int driveTime = 20;      //this is the number of milliseconds that it takes the robot to drive 1 inch
                               //it is set so that if you tell the robot to drive forward 25 units, the robot drives about 25 inches

const int turnTime = 8;        //this is the number of milliseconds that it takes to turn the robot 1 degree
                               //it is set so that if you tell the robot to turn right 90 units, the robot turns about 90 degrees

                               //Note: these numbers will vary a little bit based on how you mount your motors, the friction of the
                               //surface that your driving on, and fluctuations in the power to the motors.
                               //You can change the driveTime and turnTime to make them more accurate

String botDirection;           //the direction that the robot will drive in (this change which direction the two motors spin in)
String distance;               //the distance to travel in each direction

/********************************************************************************/
void setup()
{
  pinMode(switchPin, INPUT_PULLUP);   //set this as a pullup to sense whether the switch is flipped

  //set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  Serial.begin(9600);           //begin serial communication with the computer

  //prompt the user to enter a command
  Serial.println("Enter a direction followed by a distance.");
  Serial.println("f = forward, b = backward, r = turn right, l = turn left");
  Serial.println("Example command: f 50");
}

/********************************************************************************/
void loop()
{
  if (digitalRead(7) == LOW)
  {                                                     //if the switch is in the ON position
    if (Serial.available() > 0)                         //if the user has sent a command to the RedBoard
    {
      botDirection = Serial.readStringUntil(' ');       //read the characters in the command until you reach the first space
      distance = Serial.readStringUntil(' ');           //read the characters in the command until you reach the second space

      //print the command that was just received in the serial monitor
      Serial.print(botDirection);
      Serial.print(" ");
      Serial.println(distance.toInt());

      if (botDirection == "f")                         //if the entered direction is forward
      {
        rightMotor(200);                                //drive the right wheel forward
        leftMotor(200);                                 //drive the left wheel forward
        delay(driveTime * distance.toInt());            //drive the motors long enough travel the entered distance
        rightMotor(0);                                  //turn the right motor off
        leftMotor(0);                                   //turn the left motor off
      }
      else if (botDirection == "b")                    //if the entered direction is backward
      {
        rightMotor(-200);                               //drive the right wheel forward
        leftMotor(-200);                                //drive the left wheel forward
        delay(driveTime * distance.toInt());            //drive the motors long enough travel the entered distance
        rightMotor(0);                                  //turn the right motor off
        leftMotor(0);                                   //turn the left motor off
      }
      else if (botDirection == "r")                     //if the entered direction is right
      {
        rightMotor(-200);                               //drive the right wheel forward
        leftMotor(255);                                 //drive the left wheel forward
        delay(turnTime * distance.toInt());             //drive the motors long enough turn the entered distance
        rightMotor(0);                                  //turn the right motor off
        leftMotor(0);                                   //turn the left motor off
      }
      else if (botDirection == "l")                   //if the entered direction is left
      {
        rightMotor(255);                                //drive the right wheel forward
        leftMotor(-200);                                //drive the left wheel forward
        delay(turnTime * distance.toInt());             //drive the motors long enough turn the entered distance
        rightMotor(0);                                  //turn the right motor off
        leftMotor(0);                                   //turn the left motor off
      }
    }
  }
  else
  {
    rightMotor(0);                                  //turn the right motor off
    leftMotor(0);                                   //turn the left motor off
  }
}
/********************************************************************************/
void rightMotor(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMA, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void leftMotor(int motorSpeed)                        //function for driving the left motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMB, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}
```

To tailor the code for our assignment, we began by removing all portions related to the sliding switch, as it was not required. We then replaced the "distance" string with "speed" and updated the prompt to allow the user to input both a direction and a speed. Below is the updated input prompt.

```c++
  //prompt the user to enter a command
  Serial.println("Enter a direction followed by a speed (low, medium, high).");
  Serial.println("f = forward, b = backward, r = turn right, l = turn left");
  Serial.println("Example command: f low");
```

Next, we updated the if/else statements to control the motors at different speeds and directions based on user input. The speeds for low, medium, and high were determined by the maximum and minimum operating speeds of the motors. This updated section of code can be seen below.

```c++
      if (botDirection == "f")                         //if the entered direction is forward
      {
        if (speed == "low")
        {
          rightMotor(80);
          leftMotor(80);
        }
        if (speed == "medium")
        {
        rightMotor(150);
        leftMotor(150);
        }
        if (speed == "high")
        {
        rightMotor(230);
        leftMotor(230);
        }
      }
      else if (botDirection == "b")                    //if the entered direction is backward
       {
        if (speed == "low")
        {
          rightMotor(-80);
          leftMotor(-80);
        }
        if (speed == "medium")
        {
        rightMotor(-150);
        leftMotor(-150);
        }
        if (speed == "high")
        {
        rightMotor(-230);
        leftMotor(-230);
        }
        }
      else if (botDirection == "r")                     //if the entered direction is right
        {
        if (speed == "low")
        {
          rightMotor(-80);
          leftMotor(80);
        }
        if (speed == "medium")
        {
        rightMotor(-150);
        leftMotor(150);
        }
        if (speed == "high")
        {
        rightMotor(-230);
        leftMotor(230);
        }
          }
      else if (botDirection == "l")                   //if the entered direction is left
      {
       if (speed == "low")
      {
        rightMotor(80);
        leftMotor(-80);
      }
      if (speed == "medium")
      {
        rightMotor(150);
        leftMotor(-150);
      }
      if (speed == "high")
      {
        rightMotor(230);
        leftMotor(-230);
      }
      }
    }
  }
}
```

We tested the program by making the motors move as if the robot were turning right, turning left, and going backward at any low, medium, and high speed.

Finally, we added the code from the distance sensor to make the motors stop when the distance measured is less than 10 cm. The reference for the prewritten code that we used and edited is at the beginnging of our final code shown below. 

```c++
/*
  SparkFun Inventor’s Kit
  Circuit 5B - Remote Control Robot

  Control a two wheeled robot by sending direction commands through the serial monitor.
  This sketch was adapted from one of the activities in the SparkFun Guide to Arduino.
  Check out the rest of the book at
  https://www.sparkfun.com/products/14326

  This sketch was written by SparkFun Electronics, with lots of help from the Arduino community.
  This code is completely free for any use.

  View circuit diagram and instructions at: https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v40
  Download drawings and code at: https://github.com/sparkfun/SIK-Guide-Code
*/

//the right motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 13;           //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;            //control pin 2 on the motor driver for the right motor
const int PWMA = 11;            //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;           //speed control pin on the motor driver for the left motor
const int BIN2 = 9;           //control pin 2 on the motor driver for the left motor
const int BIN1 = 8;           //control pin 1 on the motor driver for the left motor


//const int driveTime = 20;      //this is the number of milliseconds that it takes the robot to drive 1 inch
                               //it is set so that if you tell the robot to drive forward 25 units, the robot drives about 25 inches

const int turnTime = 8;        //this is the number of milliseconds that it takes to turn the robot 1 degree
                               //it is set so that if you tell the robot to turn right 90 units, the robot turns about 90 degrees

                               //Note: these numbers will vary a little bit based on how you mount your motors, the friction of the
                               //surface that your driving on, and fluctuations in the power to the motors.
                               //You can change the driveTime and turnTime to make them more accurate

String botDirection;           //the direction that the robot will drive in (this change which direction the two motors spin in)
String speed;               //the distance to travel in each direction

const int trigPin = 6;
const int echoPin = 7;

float duration, distance;



/********************************************************************************/
void setup()
{

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);

  //set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  Serial.begin(9600);           //begin serial communication with the computer

  //prompt the user to enter a command
  Serial.println("Enter a direction followed by a speed (low, medium, high).");
  Serial.println("f = forward, b = backward, r = turn right, l = turn left");
  Serial.println("Example command: f low");
}

/********************************************************************************/
void loop()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);


  {                                                     //if the switch is in the ON position
    if (Serial.available() > 0)                         //if the user has sent a command to the RedBoard
    {
      botDirection = Serial.readStringUntil(' ');       //read the characters in the command until you reach the first space
      speed = Serial.readStringUntil(' ');           //read the characters in the command until you reach the second space


      //print the command that was just received in the serial monitor
      Serial.print(botDirection);
      Serial.print(" ");
      Serial.println(speed);

     if (distance < 10)
      {
      rightMotor(0);                                  //turn the right motor off
      leftMotor(0);                                   //turn the left motor off
      }
  
      if (botDirection == "f")                         //if the entered direction is forward
      {
        if (speed == "low")
        {
          rightMotor(80);
          leftMotor(80);
        }
        if (speed == "medium")
        {
        rightMotor(150);
        leftMotor(150);
        }
        if (speed == "high")
        {
        rightMotor(230);
        leftMotor(230);
        }
      }
      else if (botDirection == "b")                    //if the entered direction is backward
       {
        if (speed == "low")
        {
          rightMotor(-80);
          leftMotor(-80);
        }
        if (speed == "medium")
        {
        rightMotor(-150);
        leftMotor(-150);
        }
        if (speed == "high")
        {
        rightMotor(-230);
        leftMotor(-230);
        }
        }
      else if (botDirection == "r")                     //if the entered direction is right
        {
        if (speed == "low")
        {
          rightMotor(-80);
          leftMotor(80);
        }
        if (speed == "medium")
        {
        rightMotor(-150);
        leftMotor(150);
        }
        if (speed == "high")
        {
        rightMotor(-230);
        leftMotor(230);
        }
          }
      else if (botDirection == "l")                   //if the entered direction is left
      {
       if (speed == "low")
      {
        rightMotor(80);
        leftMotor(-80);
      }
      if (speed == "medium")
      {
        rightMotor(150);
        leftMotor(-150);
      }
      if (speed == "high")
      {
        rightMotor(230);
        leftMotor(-230);
      }
      }
    }
  }
}

/********************************************************************************/
void rightMotor(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMA, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void leftMotor(int motorSpeed)                        //function for driving the left motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMB, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}
```

### Discussion
#### Part 1
_What is the resolution of this sensing system?_

_Try to move your obstacle by a millimeter and determine qualitatively how precise it is._

We calibrated the sensor at 3.10 cm, aligning it with the end of the black section on the board. Before reaching the 3.10 cm mark, the measured distance fluctuated inconsistently between 2 cm, 4 cm, and 6 cm. When the function generator (the object we were using to measure distance) was placed very close to the sensor, the reading spiked to 2200 cm. Accuracy improved once the object was positioned at 3 cm. To test precision, we moved the function generator by 1 mm, and the sensor reading adjusted to 3.21 cm, indicating a responsive measurement. When we moved the object to 4 cm, the sensor recorded 3.94 cm, showing consistent performance. Based on these observations, the practical resolution of the system is approximately 0.1 cm (1 mm), as it reliably detects changes of this magnitude.
#### Part 2
_What is the minimum speed number for the motors to move forward?_

The minimum speed required for the motors to move forward was 28. However, this was too slow, so we set the "low" speed in the code to 80, as it remained relatively slow compared to the motors' maximum speed.

### Conclusion
Through this lab, we successfully implemented and tested an ultrasonic sensor for distance measurement and integrated it with motor control to achieve obstacle avoidance. 
By understanding the resolution and limitations of the HC-SR04 sensor, we assessed its effectiveness in detecting objects and stopping the robot when necessary.
Additionally, working with an H-Bridge motor driver allowed us to control motor direction and speed efficiently, demonstrating the importance of actuators in robotic movement. The integration of sensors and actuators in this experiment provided valuable insight into real-world applications, such as autonomous vehicles and robotic navigation systems.
Overall, this lab enhanced our understanding of sensor-based mobility and the practical challenges of implementing autonomous robotic systems.
