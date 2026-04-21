package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubSystem;


@Config
public class TurtleOpMode extends OpMode {

    DriveSubsystem drive;
    IntakeSubSystem intake;


    boolean auto; // Know if we need to run DriveToPose
    double x, y, heading; // Poses for auto
    public static double p = 0.15, i = 0, d = 0.05; // PID for drivetrain
    public static double maxSpeed; // Max speed for auto

    @Override
    public void init() {
        drive = new DriveSubsystem(telemetry, hardwareMap);
        intake = new IntakeSubSystem(telemetry,hardwareMap);

        maxSpeed = 1; // Default Speed
    }

    @Override
    public void loop() {
        // Only DriveToPose when in auto
        if (auto == true) {
            drive.driveToPose(x, y, heading);
        } else {
            //Need anything to stop drive to pose?
        }
        drive.updateOdometry();
        telemetry.addLine("Robot Pose:"+ drive.getRobotPose().getX(DistanceUnit.INCH) + "," + drive.getRobotPose().getY(DistanceUnit.INCH) + "," + drive.getRobotPose().getHeading(AngleUnit.DEGREES));

    }

    public void setTargetPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }


}
