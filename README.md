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
### Results
### Discussion
### Conclusion
