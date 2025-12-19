package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class BasicTeleOp extends TurtleOpMode {

    public void init() {
        super.init();
    }


    public void loop() {
        super.loop();

        if (gamepad1.a) {
            currentDriveMode = DriveMode.ROBOT_CENTRIC;
        } else {
            currentDriveMode = DriveMode.FIELD_CENTRIC;
        }



    }

}
