package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous
public class BlueFar extends OpMode {

    DcMotorEx frontLeft, frontRight, rearLeft, rearRight, shootMotor;
    CRServo leftUnicorn, rightUnicorn;
    GoBildaPinpointDriver pinpoint;
    int step = 0;

    @Override
    public void init() {
        setUpConfig();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 60, -12, AngleUnit.DEGREES, 180));
    }

    @Override
    public void loop() {

        pinpoint.update();
        telemetry.addLine("Step: " + step);
        telemetry.addLine("CurrentPose: \n\tX: " + pinpoint.getPosition().getX(DistanceUnit.INCH) + "\n\tY: "  + pinpoint.getPosition().getY(DistanceUnit.INCH)+ "\n\tA: "  + pinpoint.getPosition().getHeading(AngleUnit.DEGREES));

        switch (step) {

            case -1:
                shoot();
                step = 10;
                break;

            case 0:
                if (driveToPose(40, -20, -90)) {
                    step = 20;
                }
                break;


        }



    }

    public void setUpConfig() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
        rearLeft = hardwareMap.get(DcMotorEx.class, "BL");
        rearRight = hardwareMap.get(DcMotorEx.class, "BR");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        shootMotor = hardwareMap.get(DcMotorEx.class, "lw");

        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shootMotor.setDirection(DcMotorSimple.Direction.FORWARD);



        leftUnicorn = hardwareMap.get(CRServo.class, "LeftUnicorn");
        rightUnicorn = hardwareMap.get(CRServo.class, "RightUnicorn");

        leftUnicorn.setDirection(DcMotorSimple.Direction.FORWARD);
        rightUnicorn.setDirection(DcMotorSimple.Direction.REVERSE);



        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(-5.5, -5, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    }

    private boolean driveToPose (double x, double y, double a) {
        Pose2D targetPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, a), currentPose = pinpoint.getPosition();

        //Figure out the distance away from end pose
        double distanceAwayX = targetPose.getX(DistanceUnit.INCH) - currentPose.getX(DistanceUnit.INCH);
        double distanceAwayY = targetPose.getY(DistanceUnit.INCH) - currentPose.getY(DistanceUnit.INCH);
        double distanceAway = Math.sqrt(Math.pow(distanceAwayX, 2) + Math.pow(distanceAwayY, 2));
        double angleOfDistance = Math.atan2(distanceAwayY, distanceAwayX);

        //Make sure it drives in a straight line
        double translationOutput = pidCalculate(0.15, 0, 0.05, distanceAway, 0);
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
        double rotPow = -pidCalculate(.05, 0, 0, currentAngle, targetAngle);
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

    private double pidCalculate(double p, double i, double d, double currentPoint, double setpoint) {
        double error = setpoint - currentPoint;
        double power = error * p;

        return power;
    }

    // This routine drives the robot field relative
    private void fieldCentricDrive(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                pinpoint.getHeading(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        robotCentricDrive(newForward, newRight, rotate);
    }

    private void robotCentricDrive(double forward, double right, double rotate) {
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
        frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        rearLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        rearRight.setPower(maxSpeed * (backRightPower / maxPower));
    }


    private void shoot () {

        shootMotor.setPower(0.6);
        sleep(5000);
        leftUnicorn.setPower(1);
        rightUnicorn.setPower(1);
        sleep(500);
        leftUnicorn.setPower(-0.2);
        rightUnicorn.setPower(0.2);
        sleep(2000);
        leftUnicorn.setPower(1);
        rightUnicorn.setPower(1);
        sleep(500);
        leftUnicorn.setPower(-0.2);
        rightUnicorn.setPower(0.2);
        sleep(2000);
        leftUnicorn.setPower(1);
        rightUnicorn.setPower(1);
        sleep(2000);
        leftUnicorn.setPower(0);
        rightUnicorn.setPower(0);
        shootMotor.setPower(0);
    }
}
