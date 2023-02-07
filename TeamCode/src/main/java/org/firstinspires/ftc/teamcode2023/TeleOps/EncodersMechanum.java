package org.firstinspires.ftc.teamcode2023.TeleOps;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
* An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
* The names of OpModes appear on the menu of the FTC Driver Station.
* When an selection is made from the menu, the corresponding OpMode
* class is instantiated on the Robot Controller and executed.
*
* This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
* It includes all the skeletal structure that all iterative OpModes contain.
*
* Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
* Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
*/


//SCRAPED
@TeleOp(name="EncodersShit", group="Iterative Opmode")
public class EncodersMechanum extends OpMode {

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

    //Gyro
    BNO055IMU imu;



    public void initSayings() {
        String [] sayings =
        new String[] {
                "Let's Roll",
                "Ready to Rumble",
                "Beep Boop",
                "Taking over the World",
                "Fuck ms Smith",
                "I Like Caulk",
                "About to win the contest"
        };
        telemetry.addData("Status: ", sayings[(int) (Math.random() * sayings.length)]);
    }

    public void stopSayings() {
        // Create an Array of Possible Sayings the Robot can Say When it Shuts Down 
        String[] possibleSayings = new String[] { 
            "Goodbye", 
            "Sweet Dreams", 
            "Boop Beep.", 
            "No Longer Taking Over The World", 
            "Thinking About Our Win",
            "Fuck Your Life; Bing Bong",
            "Preparing for the Post-Win Party"
        }; 
        telemetry.addData("Status", possibleSayings[(int) (Math.random() * possibleSayings.length)]);
    }
    
    public void mecanumWheelsWithEncoders(double translationAngle, double translationPower, double turnPower) {
        double ADPower = translationPower * Math.sqrt(2) * 0.5 * (Math.sin(translationAngle) + Math.cos(translationAngle));
        double BCPower = translationPower * Math.sqrt(2) * 0.5 * (Math.sin(translationAngle) - Math.cos(translationAngle));

        double turningScale = Math.max(Math.abs(ADPower + turnPower), Math.abs(ADPower - turnPower));
        turningScale = Math.max(turningScale, Math.max(Math.abs(BCPower + turnPower), Math.abs(BCPower - turnPower)));

        lf.setPower((ADPower - turningScale) / turningScale);
        lb.setPower((BCPower - turningScale) / turningScale);
        rf.setPower((BCPower + turningScale) / turningScale);
        rb.setPower((ADPower + turningScale) / turningScale);
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
    public void init() {
        initSayings();
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

    }

    /*
    * Code to run ONCE after the driver hits STOP
    */
    @Override
    public void stop() {
        stopSayings();
    }

}
