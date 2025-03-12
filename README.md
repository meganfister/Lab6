# Lab 6
## Moving with Purpose: Sensing and Mobility 
Aashika Uppala, Megan Fister

3/12/25

### Introduction

### Methods
#### Instruments
•	A Computer running Arduino IDE

•	SparkFun Inventor’s kit

&nbsp; &nbsp; &nbsp; &nbsp; o	RedBoard

&nbsp; &nbsp; &nbsp; &nbsp; o	Ultrasonic sensor

&nbsp; &nbsp; &nbsp; &nbsp; o	Two motors

&nbsp; &nbsp; &nbsp; &nbsp; o	Motor Driver

#### Part 1 
First, connect the ultrasonic sensor as shown below in Figure 1.

![LED On](https://github.com/meganfister/Lab5/blob/main/Lab%205%20LED%20On.jpg)

Next, connect the RedBoard to the computer and start the Arduino IDE on the computer. Our constructed circuit is pictured below.

Using the instructions in https://projecthub.arduino.cc/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-7cabe1 for guidance, we programmed an algorithm to read the distance from the sensor and send the value over serial communications. The code we used is shown below. We changed the pin numbers to match the pins we used in our circuit.

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
Next, we used the ruler provided to test your algorithm.

a. What is the resolution of this sensing system?

b. Try to move your obstacle by a millimeter and determine qualitatively how precise it is.

We calibrated the sensor at 3.10 cm, aligning it with the end of the black section on the board. Before reaching the 3.10 cm mark, the measured distance fluctuated inconsistently between 2 cm, 4 cm, and 6 cm. When the function generator (the object we were using to measure distance) was placed very close to the sensor, the reading spiked to 2200 cm. Accuracy improved once the object was positioned at 3 cm. To test precision, we moved the function generator by 1 mm, and the sensor reading adjusted to 3.21 cm, indicating a responsive measurement. When we moved the object to 4 cm, the sensor recorded 3.94 cm, showing consistent performance. Based on these observations, the practical resolution of the system is approximately 0.1 cm (1 mm), as it reliably detects changes of this magnitude.

#### Part 2
Next, we moved the ultrasonic sensor connections on the RedBoard to pins 7 and 6. We connected the RedBoard, the Motor Driver, and the Motors as shown below in Figure 2.

Using https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v40/circuit-5b-remote-controlled-robot for guidance, we wrote a program to control the movement of both motors.

Insert commands via serial port to move both motors at 3 different speeds (slow, medium, fast). Document all code.

Questions: What is the minimum speed number for the motors to move forward?

4. Test the program by making the motors move as if the robot were turning right, turning left, and going backward at any speed.

5. Include the code from the distance sensor to make the motors stop when the distance measured is less than 10cm. Demonstrate your code to the instructor.


```c++
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

### Results
### Discussion
### Conclusion
