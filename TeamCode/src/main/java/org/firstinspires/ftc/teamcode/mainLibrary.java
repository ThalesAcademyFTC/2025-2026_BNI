package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;


public class mainLibrary {
    //This class will act as our "goldfish" or "robot" class for the year
    public HardwareMap hwMap;

    public LinearOpMode auton;
    public double ElapsedTime;

    public enum Drivetrain {

        MECHANUM,

        TEST

    }

    public Drivetrain drive;

    public Telemetry telemetry;

    //defines motors to all classes that call main/mainlibrary

    public DcMotorEx motorFL, motorFR, motorBL, motorBR, cannonMotor1, cannonMotor2, intakeMotor;

    public Servo THESERVO;

    public static DcMotorEx[] allMotors;

    public Servo rgbIndicator;

    public RevColorSensorV3 colorSensor;

    public TouchSensor touchSensor;

    public Rev2mDistanceSensor distanceSensor;

    public WebcamName camera;

    public IMU imu;

    public String motifPattern;

    //for commit

    public Rev9AxisImu ohMyCog;

    public boolean leftToggleToggle = true;

    public boolean shotPossibility = false;

    public double tagOrientationX;
    public double tagOrientationY;
    public double tagOrientationZ;



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

        setUpHardware();
    }

    public void setUpHardware() {
        switch (drive) {
            case MECHANUM:
                //motors gotted in hardware map

                motorFL = (DcMotorEx) hwMap.dcMotor.get("motorFL");
                motorFR = (DcMotorEx) hwMap.dcMotor.get("motorFR");
                motorBL = (DcMotorEx) hwMap.dcMotor.get("motorBL");
                motorBR = (DcMotorEx) hwMap.dcMotor.get("motorBR");
                cannonMotor1 = (DcMotorEx) hwMap.dcMotor.get("cannonMotor1");
                cannonMotor2 = (DcMotorEx) hwMap.dcMotor.get("cannonMotor2");
                intakeMotor = (DcMotorEx) hwMap.dcMotor.get("intakeMotor");
                THESERVO = hwMap.servo.get("THESERVO");

                //If a motor direction needs to be flipped:
                motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
                motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
                //motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
                //motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

                //servos gotted in hardware map

                //rgbIndicator = hwMap.servo.get("rgbIndicator");

                //sensors defined here

                //touchSensor =  hwMap.touchSensor.get("TouchSensor");
                //distanceSensor = hwMap.get(Rev2mDistanceSensor.class,"DistanceSensor");
                //colorSensor = hwMap.get(RevColorSensorV3.class,"ColorSensor");

                IMU imu = hwMap.get(IMU.class, "imu");

                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
                // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
                imu.initialize(parameters);

                //all motors defined here

                allMotors =  new DcMotorEx[] {motorBL, motorBR, motorFL, motorFR};

                break;
        }
        switch (drive) {
            case TEST:
                //pass
                break;
        }

    }


    public void resetDriveEncoders() {

        for (DcMotorEx x : allMotors) {

            x.setPower(0);
            x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            x.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void waitForMotors() {
        boolean finished = false;
        while (auton.opModeIsActive() && !finished && !auton.isStopRequested()) {
            if (motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy()) {
                telemetry.addData("front left encoder:", "%7d / % 7d", motorFL.getCurrentPosition(), motorFL.getTargetPosition());
                telemetry.addData("back left encoder:", "%7d / % 7d", motorBL.getCurrentPosition(), motorBL.getTargetPosition());
                telemetry.addData("front right encoder:", "%7d / % 7d", motorFR.getCurrentPosition(), motorFR.getTargetPosition());
                telemetry.addData("back right encoder:", "%7d / % 7d", motorBR.getCurrentPosition(), motorBR.getTargetPosition());
                telemetry.update();
            } else {
                finished = true;
            }
        }
    }

}
//

