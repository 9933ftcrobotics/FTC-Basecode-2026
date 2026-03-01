package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueClose extends TurtleOpMode {

    int step = 0;

    @Override
    public void init() {

        super.init();
        //drive.seedPose(0,0,0);  <- Null Pointer Exception?
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addLine("Step: " + step);

        switch (step) {


            case 0:

                drive.seedPose(0,0,0);
                step = 10;
                break;
            case 10:

                intake.start();
                drive.driveToPose(12, 12, 90);
                if (drive.isRobotAtTarget()) {
                    step = 20;
                }
                break;
            case 20:
                intake.stop();
                break;

        }



    }

}
