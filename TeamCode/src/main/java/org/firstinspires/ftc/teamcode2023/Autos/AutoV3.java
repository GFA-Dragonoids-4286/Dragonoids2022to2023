package org.firstinspires.ftc.teamcode2023.Autos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Math;

@Autonomous(name="AutoV3", group="Autonomous")
public class AutoV3 extends LinearOpMode {
  private ElapsedTime runtime = new ElapsedTime();

  //Wheels
  private DcMotor lf = null;
  private DcMotor lb = null;
  private DcMotor rf = null;
  private DcMotor rb = null;

  // Use Encoders
  public float currentlf = 0.0f;
  public float currentrf = 0.0f;
  public float currentlb = 0.0f;
  public float currentrb = 0.0f;

  //gyro
  BNO055IMU imu;
  Orientation angles = null;


  //motorData
  static final double MotorRevolution = 288;
  static final double DriveGearReduction = 4;
  static final double wheelWidth = 4; //inches
  static final double countsPerInch = (MotorRevolution * DriveGearReduction) / (wheelWidth * 3.1415);


  public void driveStraight(int distance, double speed) {
    lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    lf.setTargetPosition((int) Math.rint(countsPerInch * distance));
    lb.setTargetPosition((int) Math.rint(countsPerInch * distance));
    rf.setTargetPosition((int) Math.rint(countsPerInch * distance));
    rb.setTargetPosition((int) Math.rint(countsPerInch * distance));

    lf.setPower(speed);
    lb.setPower(speed);
    rf.setPower(speed);
    rb.setPower(speed);
  }

  public void halt() {
    double speed = 0;

    lf.setPower(speed);
    lb.setPower(speed);
    rf.setPower(speed);
    rb.setPower(speed);
  }

  public void turn(int degrees, double speed) {
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    double currentHeading = angles.firstAngle;
    double newHeading = currentHeading + degrees; //need to make sure that its not over 360
    if(newHeading > 360) {
      newHeading -= 360;
    }

    if(currentHeading>= newHeading) {
      while (!(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle == newHeading)) {
        lf.setPower(speed);
        lb.setPower(speed);
        rf.setPower(-speed);
        rb.setPower(-speed);
      }
    }
    else if(currentHeading <= newHeading) {
      while (!(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle == newHeading)) {
        lf.setPower(-speed);
        lb.setPower(-speed);
        rf.setPower(speed);
        rb.setPower(speed);
      }
    }
  }

  public void initWheels() {

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

    // Reset the Encoder Values
    // REASON: Fix the encoders.
    lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Make it so that if there is no power to motors, they break.
    // REASON: Makes the robot stop much faster.
    rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Set Current Encoder Position
    currentlf = lf.getCurrentPosition();
    currentrf = rf.getCurrentPosition();
    currentlb = lb.getCurrentPosition();
    currentrb = rb.getCurrentPosition();

    // Set Encoder Position
    lf.setTargetPosition((int) currentlf);
    rf.setTargetPosition((int) currentrf);
    lb.setTargetPosition((int) currentlb);
    rb.setTargetPosition((int) currentrb);

    // Make the Motors so they run using the Encoder
    // REASON: This Leads To More Dependable Movement/ We are Now Able to Track Our Movement
    lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    // Set the Power so Op modes Hopefully Work
    lf.setPower(0.1f);
    lb.setPower(0.1f);
    rf.setPower(0.1f);
    rb.setPower(0.1f);
  }

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

  @Override
  public void runOpMode() throws InterruptedException{
      //hardware stuff
      initWheels();
      initIMU();

    waitForStart();

    //run stuff

    
}

}
