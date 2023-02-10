package org.firstinspires.ftc.teamcode2023.Autos.AutoV3;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

public class AccelerationIMU {

    public double speedX = 0.00;
    public double speedY = 0.00;
    RobotoHardware robot = new RobotoHardware();

    public double getXAcceleration() {
        Acceleration acceleration = robot.imu.getAcceleration();

        return acceleration.xAccel;
    }

    public double getYAcceleration() {
        Acceleration acceleration = robot.imu.getAcceleration();

        return acceleration.yAccel;
    }

    public void update() {

    }

}
