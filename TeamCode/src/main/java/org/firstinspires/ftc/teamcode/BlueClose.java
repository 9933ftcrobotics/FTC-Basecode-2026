package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueClose extends TurtleOpMode {

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
                drive.seedPose(0,0,0);
                step = 10;
                break;
            case 10:

                intake.start();
                setTargetPose(12, 0, 0);
                if (drive.isRobotAtTarget()) {
                    step = 20;
                }
                break;
            case 20:
                setTargetPose(12, 12, 0);
                if (drive.isRobotAtTarget()) {
                    step = 30;
                }
                break;
            case 30:
                setTargetPose(0, 12, 0);
                if (drive.isRobotAtTarget()) {
                    step = 40;
                }
                intake.stop();
                break;
            case 40:
                setTargetPose(0, 0, 0);
                if (drive.isRobotAtTarget()) {
                    step = 50;
                }
                break;
                }

        }



    }


