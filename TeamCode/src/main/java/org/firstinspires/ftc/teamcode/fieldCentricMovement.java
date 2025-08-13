package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.hardware.IMU;


public class fieldCentricMovement {



    public void fieldCentricMovement(double y, double x, double rx){
        
        /* This stuff goes in teleop 
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1; //stick drift :(
        double rx = gamepad1.right_stick_x;
        */

        frontLeftMotor.setPower(y + x + rx); 
        backLeftMotor.setPower(y - x + rx);
        frontRightMotor.setPower(y - x - rx);
        backRightMotor.setPower(y + x - rx);
    }

}