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
                    timer.reset();
                    step = 20;
                }
                break;
            case 10:

                drive.setTargetPose(36, 0, 0);
                if (drive.isRobotAtTarget()) {
                    timer.reset();
                    step = 15;
                }
                break;
            case 15:
                if(timer.seconds() > 0.5) {
                    step = 4;
                }
                break;
            case 20:
                if(timer.seconds() > 1.0) {
                    step = 10;
                }
                break;

        }   }
}