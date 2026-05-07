package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class OdometrySubsystem {

    Limelight3A limelight;
    Telemetry telemetry;
    GoBildaPinpointDriver pinpoint;

    public OdometrySubsystem(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(6.5, -2.25, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();
    }

    public Pose2D getRobotPose() {
        return getPinpointPose();
    }

    ////////////////////// Pinpoint Methods //////////////////////

    public void updateOdometry() {
        Pose2D visionPose = getCombinedPose();
        pinpoint.update();

        if (visionPose != null) {
            //seedVisionPose(visionPose);
        }
    }

    public void seedPose(double x, double y, double degrees) {
        seedPose(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, degrees));
    }

    public void seedPose(Pose2D newPose) {
        pinpoint.setPosition(newPose);
    }

    public void seedVisionPose(Pose2D newPose) {
        Pose2D averagedPose = new Pose2D(
                DistanceUnit.INCH,
                (((getRobotPose().getX(DistanceUnit.INCH) * 4) + newPose.getX(DistanceUnit.INCH)) / 5),
                (((getRobotPose().getY(DistanceUnit.INCH) * 4) + newPose.getY(DistanceUnit.INCH)) / 5),
                AngleUnit.DEGREES,
                (((getRobotPose().getHeading(AngleUnit.DEGREES) * 4) + newPose.getHeading(AngleUnit.DEGREES)) / 25)
                );
        pinpoint.setPosition(averagedPose);
    }

    private Pose2D getPinpointPose() {
        return pinpoint.getPosition();
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
        limelight.updateRobotOrientation(getPinpointPose().getHeading(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();
                return new Pose2D(
                        DistanceUnit.METER, botpose.getPosition().x, botpose.getPosition().y,
                        AngleUnit.DEGREES, getPinpointPose().getHeading(AngleUnit.DEGREES)
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
