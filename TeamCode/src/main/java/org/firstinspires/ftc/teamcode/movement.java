package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class movement {

    public mainLibrary mainLibrary;

    public double tickPerInch = 50;

    public double tickPerDegree = 0;

    public driverCentricMovement driverCentricMovement;

    public movement(mainLibrary mainLibrary, driverCentricMovement driverCentricMovement) {

        this.mainLibrary = mainLibrary;

        this.driverCentricMovement = driverCentricMovement;

    }

    public void cannonLaunch() {
        mainLibrary.cannonMotor.setPower(1);
    }

    public void cannonStop() {
        mainLibrary.cannonMotor.setPower(0);
    }

    public void primeLaunch() {
        mainLibrary.THESERVO.setPosition(.45);
    }

    public void restTHESERVO() {
        mainLibrary.THESERVO.setPosition(0.95);
    }

    public void moveForward(double inches, double speed) {

        int TickTarget = (int) Math.round(inches * tickPerInch);

        mainLibrary.resetDriveEncoders();

        mainLibrary.motorFL.setTargetPosition(TickTarget);
        mainLibrary.motorFR.setTargetPosition(TickTarget);
        mainLibrary.motorBL.setTargetPosition(TickTarget);
        mainLibrary.motorBR.setTargetPosition(TickTarget);

        driverCentricMovement.driverMovement(0, speed, 0);

        for (DcMotorEx x : mainLibrary.allMotors) {

            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }


        mainLibrary.waitForMotors();

        mainLibrary.resetDriveEncoders();

    }

    public void moveBackward(double inches, double speed) {

        moveForward(-inches, speed);

    }

    public void moveRight(double inches, double speed) {

        int TickTarget = (int) Math.round(inches * tickPerInch);

        mainLibrary.resetDriveEncoders();

        mainLibrary.motorFL.setTargetPosition(TickTarget);
        mainLibrary.motorFR.setTargetPosition(-TickTarget);
        mainLibrary.motorBL.setTargetPosition(-TickTarget);
        mainLibrary.motorBR.setTargetPosition(TickTarget);

        driverCentricMovement.driverMovement(speed, 0, 0);

        for (DcMotor x : mainLibrary.allMotors) {

            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }


        mainLibrary.waitForMotors();

        mainLibrary.resetDriveEncoders();

    }
    public void moveLeft(double inches, double speed) {

        moveRight(-inches, speed);

    }

    public void turnRight(double degrees, double speed) {

        int TickTarget = (int) Math.round(degrees * tickPerDegree);

        mainLibrary.resetDriveEncoders();

        mainLibrary.motorFL.setTargetPosition(TickTarget);
        mainLibrary.motorFR.setTargetPosition(TickTarget);
        mainLibrary.motorBL.setTargetPosition(-TickTarget);
        mainLibrary.motorBR.setTargetPosition(-TickTarget);

        driverCentricMovement.driverMovement(0,0,speed);

        for (DcMotor x : mainLibrary.allMotors) {

            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }


        mainLibrary.waitForMotors();

        mainLibrary.resetDriveEncoders();

    }

    public void turnLeft(double degrees, double speed) {

        turnRight(-degrees, speed);

    }
}
