package org.firstinspires.ftc.teamcode2023.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import java.util.Math;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop
 * period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 */

@TeleOp(name = "WindsorDrive", group = "Iterative Opmode")

public class NewDrive2 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Wheels
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private double slowFactor = 1;

    //Slider Servo
    private DcMotor slider;
    private final static double SLIDER_HOME = 1;
    private double sliderPosition = SLIDER_HOME;
    private final double SLIDER_SPEED = 1;

    //Claw Servo
    private Servo claw;

    private final static double CLAW_HOME = 0.4;
    private double clawPosition = CLAW_HOME;
    private final double CLAW_SPEED = 0.2;

    public void brianMecanumWheels(double leftStickY, double leftStickX, double rightStickX) {
        double max;

        double axial = -leftStickY / 0.5; // pushing stick forward gives negative value ** it is
        // diveded to match the sideways speed with the forward and back
        // speed
        double lateral = leftStickX / 0.5;
        double yaw = rightStickX / 0.5; // Note: divided to make rotation slower

        // Combine the joystick requests for each axis-motion to determine each wheel's
        // power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        //checks to make sure that the max is not exeded the most motor power
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2df", leftBackPower, rightBackPower);

    }

    /*
    public void brianDriveSlider(double rightTrigger, double leftTrigger) {

        double power = (rightTrigger - leftTrigger)/2; //adds power to get what the user wants

        double targetPosition = slider.getPosition() + power;

        slider.setPosition(targetPosition);
        //telemetry.addData("Slider targetPosition:" + String(targetPosition));
    }
    */

    public void brianDriveClaw(boolean a, boolean b) {
        if (a){
            clawPosition = 0.4;
        } else if (b) {
            clawPosition = 0.7;
        }
        claw.setPosition(clawPosition);


    }

    public void yetAnotherMecanumWheels() {
        double y = -gamepad1.left_stick_y / slowFactor;
        double x = gamepad1.left_stick_x / slowFactor;
        double rx = gamepad1.right_stick_x / slowFactor;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flp = (y + x + rx) / denominator;
        double blp = (y - x + rx) / denominator;
        double frp = (y - x - rx) / denominator;
        double brp = (y + x - rx) / denominator;

        leftFront.setPower(flp);
        leftBack.setPower(blp);
        rightFront.setPower(frp);
        rightBack.setPower(brp);
    }

    public void slowFast() {
        if (gamepad1.left_bumper) {
            slowFactor = 2;
        }
        else if (gamepad1.right_bumper) {
            slowFactor = 1;
        }
    }

    public void neilMecanumWheels(){
        double horizontal = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;
        double vertical = gamepad1.left_stick_y;

        rightFront.setPower(0.7*(-pivot+(vertical-horizontal)));
        rightBack.setPower(0.7*(-pivot+vertical+horizontal));
        leftFront.setPower(0.7*(pivot+vertical+horizontal));
        leftBack.setPower(0.7*(pivot+(vertical-horizontal)));
    }
    public void neilDriveClaw(boolean a, boolean b){
        if (a){
            clawPosition += 0.01;
        } else if (b) {
            clawPosition -= 0.01;
        }
        claw.setPosition(clawPosition);
    }
    /*
    public void neilDriveSlider(double leftTrig, double rightTrig){
        if (rightTrig >= 0.2){
            sliderPosition += rightTrig/SLIDER_SPEED;  //this will set a position for the servo between 0 and 1 so i think slider spead needs to be lower
        } else if (leftTrig >= 0.2) {
            sliderPosition -= leftTrig/SLIDER_SPEED;
        }
        slider.setPosition(sliderPosition);
    }
    */
    public void motorSlider(double righttrig, double lefttrig) {
        double powerToMotor = (righttrig-lefttrig) * SLIDER_SPEED;

        slider.setPower(powerToMotor);
    }

    public void initWheels() {
        //gets Wheels
        leftFront = hardwareMap.get(DcMotor.class, "flm");
        leftBack = hardwareMap.get(DcMotor.class, "blm");
        rightFront = hardwareMap.get(DcMotor.class, "frm");
        rightBack = hardwareMap.get(DcMotor.class, "brm");

        //Sets Direction of Wheels
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Make it so that if there is no power to motors, they break.
        // REASON: Makes the robot stop much faster.
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void initSlider() {
        slider = hardwareMap.get(DcMotor.class, "slider");

    }

    public void initClaw() {
        claw = hardwareMap.servo.get("claw");
        claw.setPosition(CLAW_HOME);

    }
    public void initMessages() {
        String[] possibleSayings =
                new String[] {
                        "Let's roll.",
                        "Ready To Rumble.",
                        "Beep Boop.",
                        "Taking Over The World",
                        "About to Win The Contest",
                        "Don't get stuck",
                        "I Bust Nuts",
                        "Somthings Broken...",
                        "Fuck Mrs. Smith"
                };

        telemetry.addData("Status", possibleSayings[(int) (Math.random() * possibleSayings.length)]);
    }

    @Override
    public void init() {
        initWheels();
        initSlider();
        initClaw();
        initMessages();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        neilMecanumWheels();
        //neilDriveSlider(gamepad1.left_trigger, gamepad1.right_trigger);
        motorSlider(gamepad2.right_trigger, gamepad2.left_trigger);
        //neilDriveClaw(gamepad1.a, gamepad1.b);
        brianDriveClaw(gamepad1.a, gamepad1.b);


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // Create an Array of Possible Sayings the Robot can Say When it Shuts Down
        String[] possibleSayings =
                new String[] {
                        "Goodbye",
                        "Sweet Dreams",
                        "Boop Beep.",
                        "No Longer Taking Over The World",
                        "Thinking About Our Win",
                        "Preparing for the Post-Win Party"
                };
        telemetry.addData("Status", possibleSayings[(int) (Math.random() * possibleSayings.length)]);
    }

}
