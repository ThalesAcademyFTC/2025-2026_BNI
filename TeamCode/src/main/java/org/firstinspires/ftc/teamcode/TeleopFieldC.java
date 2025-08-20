package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopFieldC {
    public fieldCentricMovement fieldCentricMovement;

    public void init() {

    }
    public void loop() {
        fieldCentricMovement.fieldMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
