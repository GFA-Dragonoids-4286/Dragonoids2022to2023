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
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two
 * wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code
 * folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver
 * Station OpMode list
 */

@TeleOp(name = "MechyWheels Work", group = "Iterative Opmode")

public class MecanumWheelsWorking extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private Servo slider = null;
    private Servo claw = null;

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
        double openValue = 0.75;
        double closeValue = 0.1;

        if(open) {
            claw.setPosition(openValue);
        } 
        else if(close) {
            claw.setPosition(closeValue);
        }
        
    }

    public void initWheels() {
        leftFront = hardwareMap.get(DcMotor.class, "flm");
        leftBack = hardwareMap.get(DcMotor.class, "blm");
        rightFront = hardwareMap.get(DcMotor.class, "frm");
        rightBack = hardwareMap.get(DcMotor.class, "brm");
        slider = hardwareMap.get(DcMotor.class, "Slider")

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        
    }

    public void initSlider() {
        slider = hardwareMap.get(Servo.class, "slider");
    }

    public void initArm() {
        claw = hardwareMap.get(Servo.class, "claw")
    }

    public void initMessages() {
        String[] possibleSayings =
        new String[] {
          "Let's roll.",
          "Ready To Rumble.",
          "Beep Boop.",
          "Taking Over The World",
          "About to Win The Contest",
          "Don't get stuck"
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
    }

}
