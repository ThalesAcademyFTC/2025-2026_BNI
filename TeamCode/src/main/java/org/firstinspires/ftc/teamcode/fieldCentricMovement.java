package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class fieldCentricMovement {

    // Retrieve the IMU from the hardware map

    // Adjust the orientation parameters to match your robot



    //we love billy dignam

    mainLibrary main;

    public fieldCentricMovement (mainLibrary mainLib){
        main = mainLib;
    }

    public void fieldMovement(double x, double y, double turn) {

        /*
        if (gamepad1.options) {
            imu.resetYaw();
        }
        */

        double botHeading = main.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double motorFLPower = (rotY + rotX + turn) / denominator;
        double motorBLPower = (rotY - rotX + turn) / denominator;
        double motorFRPower = (rotY - rotX - turn) / denominator;
        double motorBRPower = (rotY + rotX - turn) / denominator;

            /*
            divides the value of a stick's x and y value as well as another stick's turn value (can be x or y)
             by the maximum value to limit power
            */

        main.motorFL.setPower(motorFLPower);
        main.motorFR.setPower(motorFRPower);
        main.motorBL.setPower(motorBLPower);
        main.motorBR.setPower(motorBRPower);

    }
}

