package org.firstinspires.ftc.teamcode2023.Autos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMUBase extends LinearOpMode {


    private ElapsedTime runTime = new ElapsedTime();
    BNO055IMU imu;

    private Orientation lastAngles = new Orientation();
    private double currentAngle = 0;

    public void initIMU() {
        //creates parameters variable
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        //changes units for the parameters
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = 0;
    }

    public double getAngle() {
        //get current angle
         Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
         return currentAngle;

    }

    @Override
    public void runOpMode() throws InterruptedException {
    }
}
