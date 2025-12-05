package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class movement {

    public mainLibrary mainLibrary;

    public double tickPerInch = 50;

    public double tickPerDegree = 0;

    double last_time;

    double curr_time;

    ElapsedTime timer = new ElapsedTime();

    public driverCentricMovement driverCentricMovement;

    public movement(mainLibrary mainLibrary, driverCentricMovement driverCentricMovement) {

        this.mainLibrary = mainLibrary;

        this.driverCentricMovement = driverCentricMovement;

    }

    //this code should be moved to a separate class for actions, like launching in teleop or auton pls.
    public void cannonLaunch() {
        mainLibrary.cannonMotor.setPower(1);
    }

    public void cannonStop() {
        mainLibrary.cannonMotor.setPower(0);
    }

    public void primeLaunch() {
        mainLibrary.THESERVO.setPosition(0.5);
    }

    public void restTHESERVO() {
        mainLibrary.THESERVO.setPosition(0.7);
    }

    public void launchLittleBoy() {
        timer.reset();
        for (int i = 0; i < 3;) {
            cannonLaunch();
            if (timer.milliseconds() > 2000) {
                primeLaunch();
            }
            if (timer.milliseconds() > 3000) {
                restTHESERVO();
                timer.reset();
                i++;
            }
        }
        cannonStop();
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
