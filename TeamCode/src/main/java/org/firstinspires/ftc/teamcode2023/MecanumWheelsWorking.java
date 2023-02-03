package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Math;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop
 * period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 */

@TeleOp(name = "MechyWheels Work", group = "Iterative Opmode")

public class MecanumWheelsWorking extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Wheels
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    //Slider Servo
    private Servo slider = null;

    //Claw Servo
    private Servo claw = null;

    // Use Encoders
    public float currentLeftFrontValue = 0.0f;
    public float currentRightFrontValue = 0.0f;
    public float currentLeftBackValue = 0.0f;
    public float currentRightBackValue = 0.0f;

    public void mecanumWheels(double leftStickY, double leftStickX, double rightStickX) {
        double max;

        double axial = -leftStickY / 0.5; // pushing stick forward gives negative value ** it is
                                                     // diveded to match the sideways speed with the forward and back
                                                     // speed
        double lateral = leftStickX;
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
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
    }

    public void driveSlider(double triggerOutputRight, double triggerOutputLeft) {

        double power = (triggerOutputRight + -triggerOutputLeft)/10; //adds power to get what the user wants

        double targetPosition = slider.getPosition() + power;

        slider.setDirection(targetPosition);
    }

    public void driveClaw(boolean open, boolean close) {
        //sets open and close values
        double openValue = 0.75;
        double closeValue = 0.1;

        //checks buttons and opens or closes claw
        if(open) {
            claw.setPosition(openValue);
        } 
        else if(close) {
            claw.setPosition(closeValue);
        }
        
    }

    public void initWheels() {
        //gets Wheels
        leftFront = hardwareMap.get(DcMotor.class, "flm");
        leftBack = hardwareMap.get(DcMotor.class, "blm");
        rightFront = hardwareMap.get(DcMotor.class, "frm");
        rightBack = hardwareMap.get(DcMotor.class, "brm");
        slider = hardwareMap.get(DcMotor.class, "Slider")
        
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
        slider = hardwareMap.get(Servo.class, "slider");

    }

    public void initArm() {
        claw = hardwareMap.get(Servo.class, "claw");
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
        initArm();
        
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
        
        mecanumWheels(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        
        driveSlider(gamepad1.left_trigger, gamepad1.right_trigger);
        driveClaw(gamepad1.right_bumper, gamepad1.left_bumper);


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
