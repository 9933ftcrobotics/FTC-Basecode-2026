package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


public class TurtleOpMode extends OpMode {

    public DriveSubsystem drivetrain;
    public FtcDashboard dashboard;


    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        DRIVE_TO_POSE,
        ANGLE_TO_POSE
    }

    public DriveMode currentDriveMode;

    @Override
    public void init() {
        drivetrain = new DriveSubsystem(hardwareMap, telemetry);
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        drivetrain.updatePinpoint();
        if (!SystemVariables.utilizingPose) drivetrain.updateRobotPoseMT1();
        drivetrain.printCurrentPose();
        telemetry.addLine(SystemVariables.allianceColor);

        switch (currentDriveMode) {
            case ANGLE_TO_POSE:
            case DRIVE_TO_POSE:
            case FIELD_CENTRIC:
                drivetrain.teleOpDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            case ROBOT_CENTRIC:
                drivetrain.robotCentricDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            default:
                currentDriveMode = BasicTeleOp.DriveMode.FIELD_CENTRIC;
        }


        TelemetryPacket packet = new TelemetryPacket();
        //packet.fieldOverlay().setStrokeWidth(1);
        //packet.fieldOverlay().strokeCircle(0,0,9);
        packet.fieldOverlay().drawImage("/images/ftc.jpg", 24, 24, 48, 48,Math.toRadians(45), 24, 24, false);
        dashboard.sendTelemetryPacket(packet);
    }
}
