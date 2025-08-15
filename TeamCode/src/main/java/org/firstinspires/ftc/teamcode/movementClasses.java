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
//import com.qualcomm.robotcore.hardware.IMU;
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

    public mainLibrary main;

    public void moveForwardInches(double inches, double speed) {

        int TickTarget = (int) Math.round(inches* main.tickPerInch);

        //main.resetDriveEncoders();

        main.motorFL.setTargetPosition(TickTarget);
        main.motorFR.setTargetPosition(TickTarget);
        main.motorBL.setTargetPosition(TickTarget);
        main.motorBR.setTargetPosition(TickTarget);

        /*for (DcMotor x : allMotors) {

            x.setMode(dcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
*/
        main.driverCentricMovement(speed , 0 , 0);

       /* for (DcMotor x : allMotors) {

            x.setMode(DcMotor.RunMode.RUN_TO_POSITION)

        }
*/
       // main.waitForMotors();

       // main.resetDriveEncoders();

    }

}