package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {

    // PIDF Coefficients
    private double Kp;
    private double Ki;
    private double Kd;
    private double Kf;
    ElapsedTime timer;
    double integralSum = 0;
    double error;
    double derivitive;
    double lastError = 0;
    // Amount of allowable error
    private double tolerance;

    private double output;
    // Add other class instance variables as needed


    // This constructor creates the PIDF object and sets its coefficients.
    // This runs whenever your code runs new PIDF() with four numbers as parameters.
    public PIDF(double Kp, double Ki, double Kd, double Kf, double tolerance) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.tolerance = tolerance;
        timer = new ElapsedTime();
    }

    // This is the main PIDF loop it should be run in an OpMode inside a while loop
    public double update(double target, double currentValue) {

        // Using the sample code from the last assignment as a guide
        // implement the PIDF control loop to generate the value of output.
        //
        // Note 1: Since this update method will be called from inside a while loop
        // you should not use a while loop here.  Instead, use an if/else to check
        // whether the error is smaller than the tolerance.
        //
        // Note 2: You will have to create a few other instance variable.  Make sure
        // to initialize them above.
        error = target - currentValue;
        if(Math.abs(error) > tolerance){
            derivitive = (error - lastError)/timer.seconds();
            integralSum = integralSum + (error * timer.seconds());
            output = (integralSum*Ki)+(derivitive*Kd)+(error*Kp)+Kf;
            lastError = error;
            timer.reset();
        }
        else{
            output = Kf;
        }

        return output;

    }

}
