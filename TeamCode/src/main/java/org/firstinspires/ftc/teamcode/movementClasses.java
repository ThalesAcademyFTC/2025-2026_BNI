package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import android.graphics.Path;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

public class movementClasses {

    public mainLibrary mainLibrary;

    public double tickPerInch = 50;

    public driverCentricMovement driverCentricMovement;

    public movementClasses(mainLibrary mainLibrary, driverCentricMovement driverCentricMovement) {

        this.mainLibrary = mainLibrary;

        this.driverCentricMovement = driverCentricMovement;

    }

    public void moveForwardInches(double inches, double speed) {

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

    public void moveBackwardInches(double inches, double speed) {

        moveForwardInches(-inches, speed);

    }

    public void moveRightInches(double inches, double speed) {

        int TickTarget = (int) Math.round(inches * tickPerInch);

        mainLibrary.resetDriveEncoders();

        mainLibrary.motorFL.setTargetPosition(TickTarget);
        mainLibrary.motorFR.setTargetPosition(-TickTarget);
        mainLibrary.motorBL.setTargetPosition(-TickTarget);
        mainLibrary.motorBR.setTargetPosition(TickTarget);

        for (DcMotor x : mainLibrary.allMotors) {

            x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        driverCentricMovement.driverMovement(speed, 0, 0);

        for (DcMotor x : mainLibrary.allMotors) {

            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }


        mainLibrary.waitForMotors();

        mainLibrary.resetDriveEncoders();

    }

    public void moveLeftInches(double inches, double speed) {

        moveRightInches(-inches, speed);

    }
}
