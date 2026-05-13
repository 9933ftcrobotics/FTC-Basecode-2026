package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous
public class blueclUHose extends TurtleOpMode {

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

        switch (step) {


            case 0:
                drive.seedPose(0, 0, 0);
                step = 4;
                break;

            case 4:
                drive.setTargetPose(0, 0, 0);
                if (drive.isRobotAtTarget()) {
                    step = 10;
                }
                break;
            case 10:

                drive.setTargetPose(36, 0, 0);
                if (drive.isRobotAtTarget()) {
                    step = 4;
                }
                break;

        }   }
}