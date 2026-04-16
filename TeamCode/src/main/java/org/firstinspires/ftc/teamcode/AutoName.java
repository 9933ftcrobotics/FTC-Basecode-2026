package org.firstinspires.ftc.teamcode;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoName extends TurtleOpMode {

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
                drive.seedPose(0, 0, 0);
                step = 10;
                break;
            case 10:
                drive.driveToPose(70, 6, -40);
                if (drive.isRobotAtTarget()) {
                    step = 20;
                }
                break;
            case 20:
                intake.start();
                shooter.setShooterSpeed(.9);
                sleep(3000);
                step = 30;
                break;
            case 30:
                shooter.setLoading(.25);
                delivery.setDeliverySpeed(1);
                sleep(1200);
                step = 40;
                break;
            case 40:
                shooter.setLoading(0.075);
                sleep(1200);
                step = 50;
                break;
            case 50:
                shooter.setLoading(.25);
                delivery.setDeliverySpeed(1);
                sleep(1200);
                step = 60;
                break;
            case 60:
                shooter.setLoading(0.075);
                sleep(1200);
                step = 70;
                break;
            case 70:
                shooter.setLoading(.25);
                delivery.setDeliverySpeed(1);
                sleep(1200);
                step = 80;
                break;
            case 80:
                shooter.setLoading(0.075);
                sleep(1200);
                step = 90;
                break;
            case 90:
                drive.driveToPose(80,23,0);
                if (drive.isRobotAtTarget()) {
                    step = 100;
                }
                break;
            case 100:
                //sleep(4000);
                intake.start();
                sleep(1000);
                step = 110;
                break;
            case 110:
                drive.driveToPose(109,19,0);
                if (drive.isRobotAtTarget()) {
                    step = 120;
                }
                break;
            case 120:
                drive.driveToPose(70, 6, -40);
                if (drive.isRobotAtTarget()) {
                    step = 130;
                }
                break;
            case 130:
                intake.start();
                shooter.setShooterSpeed(.9);
                sleep(3000);
                step = 140;
                break;
            case 140:
                shooter.setLoading(.25);
                delivery.setDeliverySpeed(1);
                sleep(1200);
                step = 150;
                break;
            case 150:
                shooter.setLoading(0.075);
                sleep(1200);
                step = 160;
                break;
            case 160:
                shooter.setLoading(.25);
                delivery.setDeliverySpeed(1);
                sleep(1200);
                step = 170;
                break;
            case 170:
                shooter.setLoading(0.075);
                sleep(1200);
                step = 180;
                break;
            case 180:
                shooter.setLoading(.25);
                delivery.setDeliverySpeed(1);
                sleep(1200);
                step = 190;
                break;
            case 190:
                shooter.setLoading(0.075);
                sleep(1200);
                step = 300;
                break;
            case 300:
                shooter.setShooterSpeed(0);
                intake.stop();
                delivery.setDeliverySpeed(0);
        }

    }

}


