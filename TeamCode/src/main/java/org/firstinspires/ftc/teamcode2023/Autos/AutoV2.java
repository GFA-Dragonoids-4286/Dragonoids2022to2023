package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.SimpleDateFormat;
import java.util.Date;

@Autonomous(name="AutoV2", group="Autonomous")
public class AutoV2 extends LinearOpMode {

  private Gyroscope imu;
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor lf = null;
  private DcMotor lb = null;
  private DcMotor rf = null;
  private DcMotor rb = null;

  // Use Encoders
  public float currentLeftFrontValue = 0.0f;
  public float currentRightFrontValue = 0.0f;
  public float currentLeftBackValue = 0.0f;
  public float currentRightBackValue = 0.0f;

  public void yetAnotherMecanumWheels(double yc, double xc, double rxc) {
    double y = yc;
    double x = xc;
    double rx = rxc;

    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    double flp = (y + x + rx) / denominator;
    double blp = (y - x + rx) / denominator;
    double frp = (y - x - rx) / denominator;
    double brp = (y + x - rx) / denominator;

    lf.setPower(flp);
    lb.setPower(blp);
    rf.setPower(frp);
    rb.setPower(brp);
}
//INIT methods
public void InitWheels() {

    // Get the Motors to Drive the Movement System
    lf = hardwareMap.get(DcMotor.class, "lf");
    lb = hardwareMap.get(DcMotor.class, "lb");
    rf = hardwareMap.get(DcMotor.class, "rf");
    rb = hardwareMap.get(DcMotor.class, "rb");

    // Set the direction of the Driving Motors
    // REASON: For the Mechanim Wheels to work simply, we Invert the Left Wheels.
    lf.setDirection(DcMotor.Direction.REVERSE);
    lb.setDirection(DcMotor.Direction.REVERSE);
    rf.setDirection(DcMotor.Direction.FORWARD);
    rb.setDirection(DcMotor.Direction.FORWARD);

    // Make it so that if there is no power to motors, they break.
    // REASON: Makes the robot stop much faster.
    rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }  

  @Override
  public void runOpMode() throws InterruptedException{
    InitWheels();
    waitForStart();
    while(System.currentTimeMillis() < 500){
        yetAnotherMecanumWheels(0, 0, 0.25);
    }
    while(System.currentTimeMillis() < 1500){
        yetAnotherMecanumWheels(0.5, 0, 0);
    }
    sleep(500);
    
}

}
