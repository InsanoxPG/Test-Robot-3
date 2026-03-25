package frc.robot;

interface StateMachine {
    void update();  // updates the state based on something (usually controller values changing/every x milliseconds)
    void display(); // makes the changes happen on the robot 

}
