package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

@Autonomous
@Disabled
public class TestAuto extends OpMode {

  private Gyroscope imu;
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor leftFront = null;
  private DcMotor leftBack = null;
  private DcMotor rightFront = null;
  private DcMotor rightBack = null;


  private int leftFrontPos;
  private int leftBackPos;
  private int rightBackPos;
  private int rightFrontPos;


  @Override
  public void init() {
    imu = hardwareMap.get(Gyroscope.class, "imu");
    leftFront = hardwareMap.get(DcMotor.class, "flm");
    leftBack = hardwareMap.get(DcMotor.class, "blm");
    rightFront = hardwareMap.get(DcMotor.class, "frm");
    rightBack = hardwareMap.get(DcMotor.class, "brm");

    leftFront.setMode(DcMotor.Direction.STOP_AND_RESET_ENCODER);
    leftBack.setMode(DcMotor.Direction.STOP_AND_RESET_ENCODER);
    rightFront.setMode(DcMotor.Direction.STOP_AND_RESET_ENCODER);
    rightBack.setMode(DcMotor.Direction.STOP_AND_RESET_ENCODER);

    leftFront.setDirection(DcMotor.Direction.FORWARD);
    leftBack.setDirection(DcMotor.Direction.FORWARD);
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    rightBack.setDirection(DcMotor.Direction.REVERSE);
    telemetry.addData("Status", "Initialized");

  }

  @Override
  public void init_loop() {
  }

  @Override
  public void start() {
    runtime.reset();
  }

  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }
}
