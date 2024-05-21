M/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Manual default driving", group="Linear OpMode")
@Config
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    public static final String LoggingTag = "RRLogs";

    // Robot configuration
    private IMU     imu             = null;
    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;

    // Adjustable default values
    public static double     MAX_DRIVE_SPEED         = 0.8 ;     // Max driving speed for better distance accuracy.
    public static double     MAX_TURN_SPEED          = 0.8 ;     // Max Turn speed to limit turn rate
    public static double     P_DRIVE_GAIN            = 0.03;     // Larger is more responsive, but also less stable
    public static double     P_TURN_GAIN             = 0.02;     // Larger is more responsive, but also less stable
    public static double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.

    // Hardware default values
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.8 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    // Execution variables
    private double  driveSpeed      = 0;
    private double  turnSpeed       = 0;
    private double  targetHeading   = 0;
    private double  headingError    = 0;
    private double  leftFrontPower  = 0;
    private double  rightFrontPower = 0;
    private double  leftBackPower   = 0;
    private double  rightBackPower  = 0;
    double          axial           = 0;  // Note: pushing stick forward gives negative value
    double          lateral         = 0;
    double          yaw             = 0;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        // Initialize the motor configuration
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize the IMU configuration
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Ready to go, push INIT to start");
        telemetry.update();

        waitForStart();
        runtime.reset();

        RobotLog.ii(LoggingTag, "Start the OpMode");
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Get values from the controller.
            axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;
            boolean dpad_up     = gamepad1.dpad_up;
            boolean dpad_down   = gamepad1.dpad_down;
            boolean dpad_left   = gamepad1.dpad_left;
            boolean dpad_right  = gamepad1.dpad_right;
            // boolean gamepad_a   = gamepad1.a;
            // boolean gamepad_b   = gamepad1.b;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower  = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower   = axial - lateral + yaw;
            rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // gyro turns
            if(dpad_up)
                turnToHeading(0.0);
            if(dpad_down)
                turnToHeading(180.0);
            if(dpad_left)
                turnToHeading(90.0);
            if(dpad_right)
                turnToHeading(-90.0);

            sendTelemetry();
        }
        RobotLog.ii(LoggingTag, "End the OpMode");
    }

    private void turnToHeading(double heading) {
        RobotLog.ii(LoggingTag, "turnToHeading(): %5.1f", heading);

        double maxTurnSpeed = MAX_TURN_SPEED;

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)
                && (gamepad1.left_stick_y == 0) && (gamepad1.left_stick_x == 0) && (gamepad1.right_stick_x == 0)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry();
        }

        // Stop all motion;
        moveRobot(0, 0);
        RobotLog.ii(LoggingTag, "turnToHeading() end: current heading is %5.1f", getHeading());
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftFrontPower  = drive - turn;
        rightFrontPower = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        if (max > 1.0)
        {
            leftFrontPower /= max;
            rightFrontPower /= max;
        }

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightFrontPower);
    }

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
    */
    public void driveStraight(double distance, double heading) {
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            int leftTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            int rightTarget = rightFrontDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFrontDrive.setTargetPosition(leftTarget);
            rightFrontDrive.setTargetPosition(rightTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start driving straight, and then enter the control loop
            moveRobot(MAX_DRIVE_SPEED, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Display the various control parameters while driving
     */
    private void sendTelemetry() {
        telemetry.addData("Run Time (s): ", "%5.1f", runtime.seconds());
        telemetry.addData("Heading Target:", "%5.0f", targetHeading);
        telemetry.addData("Heading Current:", "%5.0f", getHeading());
        telemetry.addData("Front Left:", "%5.0f", leftBackPower*100);
        telemetry.addData("Front Right:", "%5.0f", rightFrontPower*100);
        telemetry.addData("Back  Left:", "%5.0f",  leftBackPower*100);
        telemetry.addData("Back  Right:", "%5.0f", rightBackPower*100);
        //telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
        //telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
        telemetry.update();

        RobotLog.dd(LoggingTag, "Heading: Target/Current: %5.1f, %5.1f - Front Left/Right: %4.1f, %4.1f - Back  Left/Right: %4.1f, %4.1f",
                targetHeading, getHeading(), leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
