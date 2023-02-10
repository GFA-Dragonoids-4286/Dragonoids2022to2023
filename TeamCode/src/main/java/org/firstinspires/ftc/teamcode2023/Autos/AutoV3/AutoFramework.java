package org.firstinspires.ftc.teamcode2023.Autos.AutoV3;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutoFramework extends LinearOpMode {

    RobotoHardware robot = new RobotoHardware();
    ElapsedTime runTime = new ElapsedTime();



    private Orientation lastAngles = new Orientation();
    private double currentAngle = 0;
    private double turnPower = 0.3;


    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = 0;
    }

    public double getAngle() {
        //get current angle
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //get difference of current angle and last angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        //normalize value
        if(deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle <= -180) {
            deltaAngle += 360;
        }

        //set good values for delta angle and orientation
        currentAngle += deltaAngle;
        lastAngles = orientation;

        telemetry.addData("gyro", orientation.firstAngle);
        telemetry.update();
        return currentAngle;

    }


    //put into new file
    public double getXAcceleration() {
        Acceleration acceleration = robot.imu.getAcceleration();

        return acceleration.xAccel;
    }

    public double getYAcceleration() {
        Acceleration acceleration = robot.imu.getAcceleration();

        return acceleration.yAccel;
    }

    public void turn(double degrees) {
        robot.deencodeMotors();
        resetAngle();

        double error = degrees;

        while(opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -turnPower : turnPower);
            robot.driveMotors(-motorPower, -motorPower, motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        robot.driveAllMotors(0);
    }

    public void turnTo(double targetDegrees) {
        robot.deencodeMotors();
        //get current angle
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //get difference of current angle and last angle
        double error = targetDegrees - orientation.firstAngle;

        //normalize value
        if(error > 180) {
            error -= 360;
        } else if (error <= -180) {
            error += 360;
        }

        turn(error);
    }

    public void turnToPid(double targetDegrees) {
        robot.deencodeMotors();

        PIDControl_turning pid = new PIDControl_turning(targetDegrees, 0.01, 0, 0.003);

        while(opModeIsActive() && Math.abs(targetDegrees) - getAngle() > 2) {
            double motorPower = pid.Update(getAngle());
            robot.driveMotors(-motorPower, -motorPower, motorPower, motorPower);

        }

        robot.driveAllMotors(0);
    }

    public void turnPid(double degrees) {
        robot.deencodeMotors();
        turnToPid(degrees + getAngle());
    }

    public void drivePID(double inches) {
        robot.deencodeMotors();

        double counts = inches * robot.countsPerInch;
        PIDControl_driving pid = new PIDControl_driving(counts, 0.01, 0, 0.003);

        while(opModeIsActive() && Math.abs(counts) - robot.getAverageEncoderValues() > 2) {
            double motorPower = pid.Update(robot.getAverageEncoderValues());
            robot.driveMotors(motorPower, motorPower, motorPower, motorPower);

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
