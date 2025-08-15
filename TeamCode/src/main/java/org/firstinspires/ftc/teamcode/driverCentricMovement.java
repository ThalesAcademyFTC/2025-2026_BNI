package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.hardware.IMU;


public class driverCentricMovement {

    //we love billy dignam

    public mainLibrary mainLibrary;

    public driverCentricMovement(double x, double y, double turn) {

        //math yippeee

       double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

            double motorFLPower = (y + x + turn) / denominator;
            double motorFRPower = (y - x + turn) / denominator;
            double motorBLPower = (y - x - turn) / denominator;
            double motorBRPower = (y + x - turn) / denominator;

            /*
            divides the value of a stick's x and y value as well as another stick's turn value (can be x or y)
             by the maximum value to limit power
            */

            motorFL.setPower(motorFLPower);
            motorFR.setPower(motorFRPower);
            motorBL.setPower(motorBLPower);
            motorBR.setPower(motorBRPower);

    }

}