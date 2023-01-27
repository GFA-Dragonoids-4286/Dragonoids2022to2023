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

@TeleOp(name="NewDrive", group="Iterative Opmode")
public class NewDrive extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor flm;
    private DcMotor blm;
    private DcMotor frm;
    private DcMotor brm;
    private DcMotor arm;
    private Servo claw;
    private Servo rotate;

    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;
    private double armPower;


    private final static double CLAW_HOME = .6;
    private double clawPosition = CLAW_HOME;
    private final double CLAW_SPEED = 0.2;

    private final static double ROTATE_HOME = 0.92;
    private double rotatePosition = ROTATE_HOME;
    private final double ROTATE_SPEED = 0.2;

    private double sin;
    private double cos;
    
    public void test(double x, double y, double turn){
        telemetry.addData("x = ", x);
        telemetry.addData("y = ", y);
        telemetry.addData("Turn = ", turn);
    }
    
    public void arm(double left_trigger) {
        if (left_trigger >= .2){
            armPower = 1;
        } else if (left_trigger < .2){
            armPower = 0;
        }
        arm.setPower(armPower);
    }
     public void armBack(double right_trigger) {
        if (right_trigger >= .2){
            armPower = -(right_trigger);
        } else if (right_trigger < .2){
            armPower = 0;
        }
        arm.setPower(armPower);
    }


    public void mecanumWheels(double x, double y, double turn){
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

        flm.setPower(leftFrontPower);
        blm.setPower(leftBackPower);
        frm.setPower(rightFrontPower);
        brm.setPower(rightBackPower);

        telemetry.addData("Theta = ", theta);
        telemetry.addData("Turn = ", turn);
        telemetry.addData("Power = ", power);

        telemetry.addData("leftFrontPower = ", leftFrontPower);
        telemetry.addData("leftBackPower = ", leftBackPower);
        telemetry.addData("rightFrontPower = ", rightFrontPower);
        telemetry.addData("rightBackPower = ", rightBackPower);
    }
    /*
    public void claw (boolean a, boolean b, boolean b2){
        if (a){
            clawPosition = 0.3;
        } else if (b) {
            clawPosition = 0.7;
        }
        else if (b2) {
            clawPosition = 0.7;
        }
        claw.setPosition(clawPosition);
    }
    */
    
    public void rotate (boolean x, boolean y){
        /*
         if (x){
             rotatePosition = .5;
         } else if (y) {
             rotatePosition = .2;
         }
         rotate.setPosition(rotatePosition);
        
        
        while(x){
            rotatePosition += 0.05;
            rotate.setPosition(rotatePosition)
        }
        stop here
        if (x){
            rotatePosition = 0.9;
        } else if (y) {
            rotatePosition = 0.2;
        }
        rotate.setPosition(rotatePosition);
        */
    }
    
    public void initMotors(){
        flm  = hardwareMap.get(DcMotor.class, "flm");
        blm = hardwareMap.get(DcMotor.class, "blm");
        frm = hardwareMap.get(DcMotor.class, "frm");
        brm = hardwareMap.get(DcMotor.class, "brm");
        //arm = hardwareMap.get(DcMotor.class, "arm");

        flm.setDirection(DcMotor.Direction.REVERSE);
        blm.setDirection(DcMotor.Direction.FORWARD); //was set to forward, now reverse
        frm.setDirection(DcMotor.Direction.REVERSE);
        brm.setDirection(DcMotor.Direction.FORWARD);
        //arm.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    /*
    public void initServos(){
        claw = hardwareMap.servo.get("claw");
        arm.setPosition(ARM_HOME);

        rotate = hardwareMap.servo.get("rotate");
        rotate.setPosition(ROTATE_HOME);
    }
    */
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        
        initMotors();
        //initServos();

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

        double left_stick_x = gamepad1.left_stick_x;
        //double left_stick_y = -gamepad1.left_stick_y;
        double left_stick_y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        boolean a = gamepad1.a;
        boolean b =  gamepad1.b;
        boolean b2 = gamepad2.b;
        boolean x = gamepad2.x;
        boolean y = gamepad2.y;

        double left_trigger = gamepad1.left_trigger;
        double right_trigger = gamepad2.right_trigger;
        
        //mecanumWheels(left_stick_x/2, left_stick_y/2, turn/2);
        
        double horizontal = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;
        double vertical = gamepad1.left_stick_y;
        double sensitivity = gamepad1.left_trigger;
        if (sensitivity > 0) {
            vertical = (float)(vertical*0.5);
            horizontal = (float)(horizontal*0.5);
            pivot = (float)(pivot*0.5);
        }
        
        /*
        frm.setPower(0.7*(-pivot+(vertical-horizontal))/1.4);
        brm.setPower(0.7*(-pivot+vertical+horizontal)/5);
        flm.setPower(0.7*(pivot+vertical+horizontal)/1.4);
        blm.setPower(0.7*(pivot+(vertical-horizontal))/5);
        */
        
        frm.setPower(0.7*(-pivot+(vertical-horizontal)));
        brm.setPower(0.7*(-pivot+vertical+horizontal));
        flm.setPower(0.7*(pivot+vertical+horizontal));
        blm.setPower(0.7*(pivot+(vertical-horizontal)));
        
        
        
        //claw(a,b,b2);
        //rotate(x, y);
        //armBack(right_trigger);
        //arm(left_trigger);
        
        test(left_stick_x, left_stick_y, turn);
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
            "Fuck Your Life; Bing Bong",
            "Preparing for the Post-Win Party"
        }; 
        telemetry.addData("Status", possibleSayings[(int) (Math.random() * possibleSayings.length)]);
    }

}
