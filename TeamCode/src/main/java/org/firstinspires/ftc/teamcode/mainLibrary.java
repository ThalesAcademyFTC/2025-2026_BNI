package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class mainLibrary {
    //This class will act as our "goldfish" or "robot class for the year
    public HardwareMap hwMap;

    public LinearOpMode auton;
    public double ElapsedTime;

    public enum Drivetrain {
        MECHANUM
    }

    public Drivetrain drive;

    public Telemetry telemetry;

    //defines motors to all classes that call Artifact

    public DcMotorEx motorFL, motorFR, motorBL, motorBR;

    public DcMotorEx allMotors;

    public Servo servo1;

    double tickPerInch = 50

    public mainLibrary(OpMode opMode, Drivetrain drive) {

        this.hwMap = opMode.hardwareMap;

        this.drive = drive;

        this.telemetry = opMode.telemetry;

        setUpHardware();
    }

    public mainLibrary(LinearOpMode opMode, Drivetrain type) {

        this.auton = opMode;

        hwMap = opMode.hardwareMap;

        telemetry = opMode.telemetry;

        drive = type;

        setUpHardware;
    }

    public mainLibrary(HardwareMap hardwareMap, Drivetrain drive) {

        this.hwMap = hardwareMap;

        this.drive = Drivetrain;

        setUpHardware;
    }
    public void setUpHardware() {
        //motors gotted in hardware map

        motorFL = (DcMotorEx) hwMap.dcMotor.get("MotorFL");
        motorFR = (DcMotorEx) hwMap.dcMotor.get("MotorFR");
        motorBL = (DcMotorEx) hwMap.dcMotor.get("MotorBL");
        motorBR = (DcMotorEx) hwMap.dcMotor.get("MotorBR");

        //servos gotted in hardware map

        servo1 = hwMap.servo.get("Servo1");

        //all motors defined here

        allMotors = new DcMotorEx[]{motorBL, motorBR, motorFL, motorFR};

        
    }

    public void resetDriveEncoders() {

        for (DcMotor x : allMotors) {

            x.setPower(0);
            x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            x.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}

