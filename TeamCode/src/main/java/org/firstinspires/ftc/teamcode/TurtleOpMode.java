package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;


public class TurtleOpMode extends OpMode {

    DriveSubsystem drive;
    IntakeSubSystem intake;
    OdometrySubsystem odometry;

    @Override
    public void init() {

        intake = new IntakeSubSystem(telemetry,hardwareMap);
        odometry = new OdometrySubsystem(telemetry,hardwareMap);
        drive = new DriveSubsystem(telemetry, hardwareMap, odometry);
    }

    @Override
    public void loop() {

        odometry.updateOdometry();
        telemetry.addLine("Robot Pose:"+ odometry.getRobotPose().getX(DistanceUnit.INCH) + "," + odometry.getRobotPose().getY(DistanceUnit.INCH) + "," + odometry.getRobotPose().getHeading(AngleUnit.DEGREES));

    }

    @Override
    public void stop() {
        odometry.endStream();
    }

}
