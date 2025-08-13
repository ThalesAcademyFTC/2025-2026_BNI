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

public class ArtifactV1 {
    //This class will act as our "goldfish" or "robot class for the year
    public HardwareMap hwMap;

    public LinearOpMode auton;
    public double ElapsedTime;

    public enum Drive {
        MECHANUM
    }

    public Drive drive;

    public Telemetry telemetry;

    //defines motors to all classes that call Artifact

    public DcMotorEx motorFL, motorFR, motorBL, motorBR;

    public DcMotorEx allMotors;

    public Servo servo1;

    public ArtifactV1(OpMode opMode, Drive drive) {

        this.hwMap = opMode.hardwareMap;

        this.drive = drive;

        this.telemetry = opMode.telemetry;

        setUpHardware();
    }
    public void setUpHardware() {
        //motors gotted in hardware map

        motorFL = (DcMotorEx) hwMap.dcMotor.get("MotorFL");
        motorFR = (DcMotorEx) hwMap.dcMotor.get("MotorFR");
        motorBL = (DcMotorEx) hwMap.dcMotor.get("MotorBL");
        motorBR = (DcMotorEx) hwMap.dcMotor.get("MotorBR");

        //servos gotted in hardware map

        servo1 = hwMap.servo.get("Servo1");










    }
}

