package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DeliverySubSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubSystem;


public class TurtleOpMode extends OpMode {

    DriveSubsystem drive;
    IntakeSubSystem intake;
    ShooterSubSystem shooter;
    DeliverySubSystem delivery;
    int tresStep = 10;
    @Override
    public void init() {

        drive = new DriveSubsystem(telemetry, hardwareMap);
        intake = new IntakeSubSystem(telemetry,hardwareMap);
        shooter = new ShooterSubSystem(telemetry,hardwareMap);
        delivery = new DeliverySubSystem(telemetry,hardwareMap);
    }

    @Override
    public void loop() {

        drive.updateOdometry();
        telemetry.addLine("Robot Pose:"+ drive.getRobotPose().getX(DistanceUnit.INCH) + "," + drive.getRobotPose().getY(DistanceUnit.INCH) + "," + drive.getRobotPose().getHeading(AngleUnit.DEGREES));

    }
    public boolean shootTres() {
        delivery.setDeliverySpeed(1);
        if (tresStep == 10) {
            intake.start();
            tresStep = 20;
        } else if (tresStep == 20) {
            shooter.setShooterSpeed(.9);
            sleep(3000);
            tresStep = 30;
        } else if (tresStep == 30) {
            shooter.setLoading(.25);
            sleep(1200);
            tresStep = 40;
        } else if (tresStep == 40) {
            shooter.setLoading(0.075);
            sleep(1200);
            tresStep = 50;
        } else if (tresStep == 50) {
            shooter.setLoading(.25);
            sleep(1200);
            tresStep = 60;
        } else if (tresStep == 60) {
            shooter.setLoading(0.075);
            sleep(1200);
            tresStep = 70;
        } else if (tresStep == 70) {
            shooter.setLoading(.25);
            sleep(1200);
            tresStep = 80;
        } else if (tresStep == 80) {
            shooter.setLoading(0.075);
            sleep(1200);
            tresStep = 90;
        } else if (tresStep == 90) {
            shooter.setShooterSpeed(0);
            intake.stop();
            delivery.setDeliverySpeed(0);
            tresStep = 10000000;
        } else {
            tresStep = 10;
            return true;
        }

        return false;
    }
}
