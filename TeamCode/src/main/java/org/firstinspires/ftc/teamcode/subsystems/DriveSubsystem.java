package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.SystemVariables;

import static org.firstinspires.ftc.teamcode.SystemVariables.DrivetrainConstants;

public class DriveSubsystem {

    private Telemetry telemetry;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    private PIDController translationalController = new PIDController(0.15, 0, 0.05);
    private PIDController rotationalController = new PIDController(0.05, 0, 0);

    private GoBildaPinpointDriver pinpoint;

    private Limelight3A limelight;

    public boolean useMT1 = true, limelightCreated = true;

    /**
     * Creates the drivetrain code
     * @param hardwareMap The active configuration
     * @param telemetry Allows drivetrain to put values on telemetry
     */
    public DriveSubsystem(
            HardwareMap hardwareMap,
            Telemetry telemetry
    ) {

        this.telemetry = telemetry;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(-39.6875, -26.19375, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();



        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
            limelight.start(); // This tells Limelight to start looking!
            limelightCreated = true;
        } catch (NullPointerException nullPointerException) {
            limelightCreated = false;
        }
    }

    public Pose2D getCurrentPose() {
        return pinpoint.getPosition();
    }


    ///////////////////////////// Driving Methods //////////////////////////////////////

    public void fieldCentricDrive(double strafePow, double forwardPow, double rotationPow) {

        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forwardPow, strafePow);
        double r = Math.hypot(strafePow, forwardPow);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                pinpoint.getHeading(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        robotCentricDrive(newForward, newRight, rotationPow);

    }

    private boolean driveToPose (double x, double y, double a) {
        Pose2D targetPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, a), currentPose = pinpoint.getPosition();

        //Figure out the distance away from end pose
        double distanceAwayX = targetPose.getX(DistanceUnit.INCH) - currentPose.getX(DistanceUnit.INCH);
        double distanceAwayY = targetPose.getY(DistanceUnit.INCH) - currentPose.getY(DistanceUnit.INCH);
        double distanceAway = Math.sqrt(Math.pow(distanceAwayX, 2) + Math.pow(distanceAwayY, 2));
        double angleOfDistance = Math.atan2(distanceAwayY, distanceAwayX);

        //Make sure it drives in a straight line
        double translationOutput = translationalController.calculateOutput(distanceAway, 0);
        translationOutput = Math.copySign(Math.min(Math.abs(translationOutput), 1), distanceAway);

        //Set New Rotation so it can cross -180
        double targetAngle = targetPose.getHeading(AngleUnit.DEGREES), currentAngle = currentPose.getHeading(AngleUnit.DEGREES);
        if (Math.abs(targetAngle - currentAngle) > 180) {
            double delta = (180 - Math.abs(targetAngle));

            if (currentAngle >= 0) {
                targetAngle = 180 + delta;
            } else {
                targetAngle = -180 - delta;
            }
        }

        //Power needed in each direction
        double rotPow = -rotationalController.calculateOutput(currentAngle, targetAngle);
        double xPow = translationOutput * Math.cos(angleOfDistance);
        double yPow = -translationOutput * Math.sin(angleOfDistance);

        telemetry.addLine("Distance Away : " + distanceAway);
        telemetry.addLine("Angle: " + angleOfDistance * 180 / Math.PI);

        //Apply power
        fieldCentricDrive(xPow, yPow, rotPow);

        if (distanceAway < 1 && Math.abs(currentAngle - targetAngle) < 5) {
            fieldCentricDrive(0, 0, 0);
            return true;
        } else {
            return false;
        }
    }

    public void teleOpDrive (double strafePow, double forwardPow, double rotationPow) {
        final double orientationOffset;
        if (SystemVariables.allianceColor == "BLUE") {
            orientationOffset = 90;
        } else {
            orientationOffset = -90;
        }

        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forwardPow, strafePow);
        double r = Math.hypot(strafePow, forwardPow);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                pinpoint.getHeading(AngleUnit.RADIANS));

        theta = AngleUnit.normalizeRadians(theta +
                AngleUnit.RADIANS.fromDegrees(orientationOffset));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        robotCentricDrive(newForward, newRight, rotationPow);

    }

    public void robotCentricDrive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        leftFront.setPower(maxSpeed * (frontLeftPower / maxPower));
        rightFront.setPower(maxSpeed * (frontRightPower / maxPower));
        leftRear.setPower(maxSpeed * (backLeftPower / maxPower));
        rightRear.setPower(maxSpeed * (backRightPower / maxPower));
    }

    //////////////////////////// Robot Heading Methods ///////////////////////////////////

    public void setAllianceColor(String color) {
        SystemVariables.allianceColor = color;
    }

    public void setUseMT1(boolean useMT1) {
        this.useMT1 = useMT1;
    }

    public void updateRobotPoseMT1() {
        LLResult result;
        if (limelightCreated) {
            result = limelight.getLatestResult();
        } else {
            result = null;
            telemetry.addLine("NOT BUILT");
        }

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double a = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
                telemetry.addData("MT1 Location", "(" + x + ", " + y + ", " + a + ")");

                x = correctedVisionOutput(getCurrentPose().getX(DistanceUnit.METER), x, DrivetrainConstants.megaTag1Trust, DistanceUnit.METER.fromInches(12));
                y = correctedVisionOutput(getCurrentPose().getY(DistanceUnit.METER), y, DrivetrainConstants.megaTag1Trust, DistanceUnit.METER.fromInches(12));
                a = correctedVisionOutput(getCurrentPose().getHeading(AngleUnit.DEGREES), a, DrivetrainConstants.megaTag1Trust, AngleUnit.DEGREES.fromDegrees(20));

                pinpoint.setPosition(new Pose2D(DistanceUnit.METER, x, y, AngleUnit.DEGREES, a));
            }
        }
    }

    public void updateRobotPoseMT2() {
        LLResult result;
        if (limelightCreated) {
            limelight.updateRobotOrientation(getCurrentPose().getHeading(AngleUnit.DEGREES));
            result = limelight.getLatestResult();
        } else {
            result = null;
        }

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");

                x = correctedVisionOutput(getCurrentPose().getX(DistanceUnit.METER), x, DrivetrainConstants.megaTag2Trust, DistanceUnit.METER.fromInches(12));
                y = correctedVisionOutput(getCurrentPose().getY(DistanceUnit.METER), y, DrivetrainConstants.megaTag2Trust, DistanceUnit.METER.fromInches(12));

                pinpoint.setPosition(new Pose2D(DistanceUnit.METER, x, y, AngleUnit.DEGREES, getCurrentPose().getHeading(AngleUnit.DEGREES)));
            }
        }
    }

    public void updatePinpoint() {
        pinpoint.update();
    }

    public void setPinpointPose(double x, double y, double a) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, a));
    }

    public void printCurrentPose() {
        telemetry.addLine("Current Pose:" + getCurrentPose().getX(DistanceUnit.INCH) + ", " +  getCurrentPose().getY(DistanceUnit.INCH) + ", " +  getCurrentPose().getHeading(AngleUnit.DEGREES));
    }




    /**
     * This method is used to prevent oscillation of pose during vision updates
     * @param prevAverage The total average of dataset, for vision use current position
     * @param input The additional input into dataset, for vision this is the vision result
     * @param trustFactor Amount previous position is trusted more, by a factor
     * @param radiusOfOscillation Range for expected oscillation, if input is outside of range, bypasses correction
     * @return The average of the dataset
     */
    public double correctedVisionOutput(double prevAverage, double input, int trustFactor, double radiusOfOscillation) {
        double average = 0;
        average = (prevAverage * trustFactor) + input;
        average /= trustFactor + 1;
        if (Math.abs(average - input) > radiusOfOscillation) return input;
        return average;
    }


}
