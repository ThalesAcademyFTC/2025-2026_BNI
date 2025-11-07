package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.hardware.IMU;


public class driverCentricMovement {

    //we love billy dignam


    //test teleop movement
    public mainLibrary main;

    public driverCentricMovement (mainLibrary mainLib){
        this.main = mainLib;
    }
    public void driverMovement(double x, double y, double turn) {

        //math yippeee
        //mess with this and experiment next meeting
       double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

            double motorFLPower = (y + x + turn) / denominator;
            //ryan's idea ^
            double motorFRPower = (y - x + turn) / denominator;
            double motorBLPower = (y - x - turn) / denominator;
            double motorBRPower = (y + x - turn) / denominator;

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