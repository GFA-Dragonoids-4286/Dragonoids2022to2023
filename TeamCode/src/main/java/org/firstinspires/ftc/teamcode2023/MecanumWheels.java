/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode2023;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@TeleOp(name="MecanumWheels", group="Iterative Opmode")
@Disabled
public class MecanumWheels extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor arm;

    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;
    private double armPower;

    private double sin;
    private double cos;
    //private double power;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        arm = hardwareMap.get(DcMotor.class, "arm");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        String [] sayings =
                new String[] {
                        "Let's Roll",
                        "Ready to Rumble",
                        "Beep Boop",
                        "Taking over the World",
                        "About to win the contest"
                };
        telemetry.addData("Status: ", sayings[(int) (Math.random() * sayings.length)]);
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

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double theta = Math.atan2(y,x);
        double power = Math.hypot(x,y);

        sin = Math.sin(theta - Math.PI / 4);
        cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin),
                Math.abs(cos));

        leftFrontPower =  power * cos/max + turn;
        rightFrontPower = power * sin/max - turn;
        leftBackPower = power * sin/max + turn;
        rightBackPower = power * cos/max - turn;

        if ((power + Math.abs(turn)) > 1){
            leftFrontPower /= power + turn;
            rightFrontPower /= power + turn;
            leftBackPower /= power + turn;
            rightBackPower /= power + turn;
        }

        if (gamepad1.left_trigger >= .2){
            armPower = gamepad1.left_trigger;
        } else if (gamepad1.left_trigger < .2){
            armPower = 0;
        }

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
        arm.setPower(armPower);

        telemetry.addData("Theta = ", theta);
        telemetry.addData("Turn = ", turn);
        telemetry.addData("Power = ", power);

        telemetry.addData("leftFrontPower = ", leftFrontPower);
        telemetry.addData("leftBackPower = ", leftBackPower);
        telemetry.addData("rightFrontPower = ", rightFrontPower);
        telemetry.addData("rightBackPower = ", rightBackPower);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Create an Array of Possible Sayings the Robot can Say When it Shuts Down 
        String[] possibleSayings = new String[] { 
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
