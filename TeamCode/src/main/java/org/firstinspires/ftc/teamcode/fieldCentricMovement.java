package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.hardware.IMU;


public class fieldCentricMovement {

    public mainLibrary main;

    public fieldCentricMovement(double y, double x, double rx) {
        
        /* This stuff goes in teleop 
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1; //stick drift :(
        double rx = gamepad1.right_stick_x;
        */

        main.motorFL.setPower(y + x + rx);
        main.motorFR.setPower(y - x + rx);
        main.motorBR.setPower(y - x - rx);
        main.motorBL.setPower(y + x - rx);
    }

}