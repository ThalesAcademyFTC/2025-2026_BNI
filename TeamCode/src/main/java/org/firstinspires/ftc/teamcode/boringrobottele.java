package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class boringrobottele extends OpMode {
    mainLibrary mainLibrary;
    driverCentricMovement driverCentricMovement;

    public void init() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.TEST);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

    }

    public void loop() {
        double x = (gamepad1.left_stick_x);
        double y = (gamepad1.left_stick_y);
        double turn = (gamepad1.right_stick_x);

        driverCentricMovement.driverMovement(x,y,turn);
    }
}
