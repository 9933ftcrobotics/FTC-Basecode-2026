package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubSystem;


public class TurtleOpMode extends OpMode {

    DriveSubsystem drive;
    IntakeSubSystem intake;

    @Override
    public void init() {

        drive = new DriveSubsystem(telemetry, hardwareMap);
        intake = new IntakeSubSystem(telemetry,hardwareMap);
    }

    @Override
    public void loop() {

        drive.updateOdometry();
        telemetry.addLine("Robot Pose:"+ drive.getRobotPose().getX(DistanceUnit.INCH) + "," + drive.getRobotPose().getY(DistanceUnit.INCH) + "," + drive.getRobotPose().getHeading(AngleUnit.DEGREES));

    }

}
