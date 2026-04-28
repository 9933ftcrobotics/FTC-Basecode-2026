package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.function.DoubleSupplier;

public class OdometrySubsystem {

    Limelight3A limelight;
    DcMotorEx xPosMotor, yPosMotor;
    Telemetry telemetry;
    Pose2D currentPose;
    DoubleSupplier xPosSupplier, yPosSupplier, rotPosSupplier;
    double prevX, prevY, prevRot;
    double xOffset = 6.5, yOffset = -2.25;
    double ticksPerInch = 1;

    public OdometrySubsystem(Telemetry telemetry, Limelight3A limelight, DcMotorEx xPosMotor, DcMotorEx yPosMotor, IMU imu) {
        this.telemetry = telemetry;
        this.limelight = limelight;

        this.xPosMotor = xPosMotor;
        this.yPosMotor = yPosMotor;

        this.xPosSupplier = xPosMotor::getCurrentPosition;
        this.yPosSupplier = yPosMotor::getCurrentPosition;

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        rotPosSupplier = () -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();
    }

    public Pose2D getRobotPose() {
        return currentPose;
    }

    ////////////////////// Pinpoint Methods //////////////////////

    public void updateOdometry() {
        Pose2D visionPose = getCombinedPose();
        updateDeadwheels();

        if (visionPose != null) {
            seedVisionPose(visionPose);
        }
    }

    private void updateDeadwheels() {
        double xPodDistance = xPosSupplier.getAsDouble() - prevX;
        double yPodDistance = yPosSupplier.getAsDouble() - prevY;

        xPodDistance /= ticksPerInch;
        yPodDistance /= ticksPerInch;

        double rotDistance = rotPosSupplier.getAsDouble() - prevRot;

        xPodDistance -= rotDistance * xOffset;
        yPodDistance -= rotDistance * yOffset;

        double newRot = AngleUnit.RADIANS.normalize(rotPosSupplier.getAsDouble() + currentPose.getHeading(AngleUnit.RADIANS));
        double newX = (xPodDistance * Math.cos(newRot)) - (yPodDistance * Math.sin(newRot));
        double newY = (xPodDistance * Math.sin(newRot)) + (yPodDistance * Math.cos(newRot));

        currentPose = new Pose2D(DistanceUnit.INCH, newX, newY, AngleUnit.RADIANS, newRot);
        prevX = xPosSupplier.getAsDouble();
        prevY = yPosSupplier.getAsDouble();
        prevRot = rotPosSupplier.getAsDouble();
    }

    public void seedPose(double x, double y, double degrees) {
        seedPose(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, degrees));
    }

    public void seedPose(Pose2D newPose) {
        currentPose = newPose;
    }

    public void seedVisionPose(Pose2D newPose) {
        currentPose = new Pose2D(
                DistanceUnit.INCH,
                (((getRobotPose().getX(DistanceUnit.INCH) * 4) + newPose.getX(DistanceUnit.INCH)) / 5),
                (((getRobotPose().getY(DistanceUnit.INCH) * 4) + newPose.getY(DistanceUnit.INCH)) / 5),
                AngleUnit.DEGREES,
                (((getRobotPose().getHeading(AngleUnit.DEGREES) * 4) + newPose.getHeading(AngleUnit.DEGREES)) / 25)
                );
    }

    ////////////////////// Limelight Methods /////////////////////

    public void endStream () {
        limelight.close();
    }

    private Pose2D getMegaTag1Pose() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                return new Pose2D(
                        DistanceUnit.METER, botpose.getPosition().x, botpose.getPosition().y,
                        AngleUnit.DEGREES, botpose.getOrientation().getYaw()
                );
            }
        }
        return null;
    }

    private Pose2D getMegaTag2Pose() {
        limelight.updateRobotOrientation(getRobotPose().getHeading(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();
                return new Pose2D(
                        DistanceUnit.METER, botpose.getPosition().x, botpose.getPosition().y,
                        AngleUnit.DEGREES, getRobotPose().getHeading(AngleUnit.DEGREES)
                );
            }
        }
        return null;
    }

    private Pose2D getCombinedPose() {

        Pose2D mt1Pose = getMegaTag1Pose(), mt2Pose = getMegaTag2Pose();

        if (mt2Pose == null) {

            return null;

        } else if (mt1Pose == null) {

            return mt2Pose;

        } else {

            return new Pose2D(
                    DistanceUnit.METER, mt2Pose.getX(DistanceUnit.METER), mt2Pose.getY(DistanceUnit.METER),
                    AngleUnit.DEGREES, mt1Pose.getHeading(AngleUnit.DEGREES)
            );

        }
    }
}
