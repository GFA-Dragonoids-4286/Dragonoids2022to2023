package org.firstinspires.ftc.teamcode2023.Autos.AutoV3;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControl_turning {

    private double targetAngle;

    //gains
    private double kP;
    private double kI;
    private double kD;

    //I
    private double accumulatedError = 0;

    //D
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double lastTime = 0;

    public PIDControl_turning(double target, double p, double i, double d) {
        targetAngle = target;
        kP = p;
        kI = i;
        kD = d;
    }

    public double Update(double currentAngle) {
        //P
        double error = targetAngle - currentAngle;

        if(error > 180) {
            error -= 360;
        } else if (error <= -180) {
            error += 360;
        }

        //I
        accumulatedError+= error;

        //resets accumulatedError if with threshold
        if (Math.abs(error) < 2) {
            accumulatedError = 0;
        }
        //makes sure sins are the same
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

        //D
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError)/(timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;

        //Final Motor
        //makes sure it is between positive 1 and 0
        double motorPower = 0.1 * Math.abs(error) + 0.9 * Math.tanh(kP * error * kI * accumulatedError * kD * slope);

        return motorPower;

    }

}
