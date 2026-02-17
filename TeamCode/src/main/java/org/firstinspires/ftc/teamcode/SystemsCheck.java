package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
@Config
public class SystemsCheck extends OpMode {

    DcMotor frontLeft, frontRight, rearLeft, rearRight, shootMotor;
    CRServo leftUnicorn, rightUnicorn;
    GoBildaPinpointDriver pinpoint;
    public static double shootpower = .6;

    @Override
    public void init() {
        setUpConfig();
    }

    @Override
    public void loop() {
        pinpoint.update();
        telemetry.addLine("Current Pose: " + pinpoint.getPosition());

        //Drive Wheel Check
        telemetry.addLine("A Button = Rear Left");
        telemetry.addLine("X Button = Front Left");
        telemetry.addLine("B Button = Rear Right");
        telemetry.addLine("Y Button = Front Right");
        if (gamepad1.a) rearLeft.setPower(0.5);
        else rearLeft.setPower(0);
        if (gamepad1.x) frontLeft.setPower(0.5);
        else frontLeft.setPower(0);
        if (gamepad1.b) rearRight.setPower(0.5);
        else rearRight.setPower(0);
        if (gamepad1.y) frontRight.setPower(0.5);
        else frontRight.setPower(0);

        //Subsystem Check
        telemetry.addLine("Dpad Up = Shooter");
        telemetry.addLine("Dpad Left = Left Unicorn");
        telemetry.addLine("Dpad Right = Right Unicorn");
        telemetry.addLine("Dpad Down = Reset Pinpoint");
        if (gamepad1.dpad_up) shootMotor.setPower(0.5);
        else shootMotor.setPower(0);
        if (gamepad1.dpad_left) leftUnicorn.setPower(0.5);
        else leftUnicorn.setPower(0);
        if (gamepad1.dpad_right) rightUnicorn.setPower(0.5);
        else rightUnicorn.setPower(0);
        if (gamepad1.dpad_down) pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
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
        frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        rearLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        rearRight.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
