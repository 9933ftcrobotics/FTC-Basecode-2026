package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous
public class NathanAuto extends TurtleOpMode {

    int step = 0;

    @Override
    public void init() {
        auto = true;
        super.init();
        //drive.seedPose(0,0,0);  <- Null Pointer Exception?
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addLine("Step: " + step);

        if (step == 0) {
            drive.seedPose(0,0,0);
            step = 10;
        }
        else if (step == 10) {
            drive.setTargetPose(0, 0, 0);
            if (drive.isRobotAtTarget()) {
                step = 20;
            }
        }
        else {

        }
    }
}