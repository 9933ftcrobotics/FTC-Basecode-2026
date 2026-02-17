package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


public class TurtleOpMode extends OpMode {

    DriveSubsystem drive;

    @Override
    public void init() {

        drive = new DriveSubsystem(telemetry, hardwareMap);

    }

    @Override
    public void loop() {

        drive.updateOdometry();

    }

}
