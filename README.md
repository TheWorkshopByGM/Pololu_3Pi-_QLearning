# Pololu_3Pi_QLearning
This is a line following code for the Pololu 3Pi+ mobile robot.
3 different controllers were implemented, a simple Bang Bang controller, a PID controller, and an advanced Q-Learning Reinforcement Learning controller.

The controllers can be found in their respective folders, please note that the codes are built from scratch (Not using the Pololu Library).

In order to launch any code, you will need to upload the Arduino Files along with the respective headers to the Pololu using the Arduino IDE.

After upload, you will need to place the robot on the line, and press the (A) button. The robot will complete 1 spin around itself to calibrate its sensors and then will start following the line.
