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

                odometry.seedPose(0,0,0);
                step = 10;
                break;
            case 10:

                intake.start();
                drive.driveToPose(12, 0, 0);
                if (drive.isRobotAtTarget()) {
                    step = 20;
                }
                break;
            case 20:
                drive.driveToPose(12, 12, 0);
                if (drive.isRobotAtTarget()) {
                    step = 30;
                }
                break;
            case 30:
                drive.driveToPose(0,12,0);
                if (drive.isRobotAtTarget()) {
                    step = 40;
                }
                intake.stop();
                break;
            case 40:
                drive.driveToPose(0,0,0);
                if (drive.isRobotAtTarget()) {
                    step = 50;
                }
                break;
        }

    }

    @Override
    public void stop() {super.stop();}

}


