package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class fieldCentricMovement {

    // Retrieve the IMU from the hardware map

    // Adjust the orientation parameters to match your robot



    //we love billy dignam

    mainLibrary mainLibrary;

    public void fieldMovement(double x, double y, double turn) {

        /*
        if (gamepad1.options) {
            imu.resetYaw();
        }
        */

        double botHeading = mainLibrary.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        //rotX = rotX * 1.1;  // Counteracts strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double motorFLPower = (rotY + rotX + turn) / denominator;
        double motorBLPower = (rotY - rotX + turn) / denominator;
        double motorFRPower = (rotY - rotX - turn) / denominator;
        double motorBRPower = (rotY + rotX - turn) / denominator;

            /*
            divides the value of a stick's x and y value as well as another stick's turn value (can be x or y)
             by the maximum value to limit power
            */

        mainLibrary.motorFL.setPower(motorFLPower);
        mainLibrary.motorFR.setPower(motorFRPower);
        mainLibrary.motorBL.setPower(motorBLPower);
        mainLibrary.motorBR.setPower(motorBRPower);

    }
}

