package org.firstinspires.ftc.teamcode2023;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="BaseOp", group="Iterative Opmode")
public class BaseTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

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
        public void start() {runtime.reset();
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
