//package org.firstinspires.ftc.robotcontroller.external.samples;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import java.text.SimpleDateFormat;
//import java.util.Date;
//import java.util.Math;
//
//@Autonomous(name="AutoV1", group="Autonomous")
//public class TestAuto extends OpMode {
//
//  private Gyroscope imu;
//  private ElapsedTime runtime = new ElapsedTime();
//  private DcMotor leftFront = null;
//  private DcMotor leftBack = null;
//  private DcMotor rightFront = null;
//  private DcMotor rightBack = null;
//
//  // Use Encoders
//  public float currentLeftFrontValue = 0.0f;
//  public float currentRightFrontValue = 0.0f;
//  public float currentLeftBackValue = 0.0f;
//  public float currentRightBackValue = 0.0f;
//
//
//
//  //INIT methods
//  public void InitWheels() {
//
//    // Get the Motors to Drive the Movement System
//    lf = hardwareMap.get(DcMotor.class, "lf");
//    lb = hardwareMap.get(DcMotor.class, "lb");
//    rf = hardwareMap.get(DcMotor.class, "rf");
//    rb = hardwareMap.get(DcMotor.class, "rb");
//
//    // Set the direction of the Driving Motors
//    // REASON: For the Mechanim Wheels to work simply, we Invert the Left Wheels.
//    lf.setDirection(DcMotor.Direction.REVERSE);
//    lb.setDirection(DcMotor.Direction.REVERSE);
//    rf.setDirection(DcMotor.Direction.FORWARD);
//    rb.setDirection(DcMotor.Direction.FORWARD);
//
//    // Reset the Encoder Values
//    // REASON: Fix the encoders.
//    lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//    // Make it so that if there is no power to motors, they break.
//    // REASON: Makes the robot stop much faster.
//    rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//    // Set Current Encoder Position
//    currentLeftFrontValue = lf.getCurrentPosition();
//    currentRightFrontValue = rf.getCurrentPosition();
//    currentLeftBackValue = lb.getCurrentPosition();
//    currentRightBackValue = rb.getCurrentPosition();
//
//    // Set Encoder Position
//    lf.setTargetPosition((int) currentLeftFrontValue);
//    rf.setTargetPosition((int) currentRightFrontValue);
//    lb.setTargetPosition((int) currentLeftBackValue);
//    rb.setTargetPosition((int) currentRightBackValue);
//
//    // Make the Motors so they run using the Encoder
//    // REASON: This Leads To More Dependable Movement/ We are Now Able to Track Our Movement
//    lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//    // Set the Power so Op modes Hopefully Work
//    lf.setPower(0.f);
//    lb.setPower(0.f);
//    rf.setPower(0.f);
//    rb.setPower(0.f);
//  }
//
//  @Override
//  public void init() {
//    initWheels();
//  }
//
//  @Override
//  public void init_loop() {
//
//  }
//
//  @Override
//  public void start() {
//    runtime.reset();
//  }
//
//  @Override
//  public void loop() {
//    telemetry.addData("Status", "Run Time: " + runtime.toString());
//  }
//}
