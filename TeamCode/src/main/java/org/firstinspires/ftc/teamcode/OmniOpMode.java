/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="OmniOpMode", group="OpMode")
public class OmniOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    // Set up to get motors
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private Servo intake = null;
    // Set up to get Odometry Hub
    private GoBildaPinpointDriver OdometryHub = null;
    // Speed Control
    private double Speed = 1.0;

    @Override
    public void runOpMode() {
        // SET UP \\
        // Get the four motors from hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR");
        // got intake servo
        intake = hardwareMap.get(Servo.class, "Intake" );
        // Set the direction of the motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Get the Odometry Hub
        OdometryHub = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");
        InitOdometry();
        OdometryHub.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until driver presses stop.
        while (opModeIsActive()) {
            double max;

            // Update Odometry
            OdometryHub.update();

            // Check for speed changes
            if (gamepad1.dpad_up) {
                Speed = Speed + 0.1;
            } else if (gamepad1.dpad_down) {
                Speed = Speed - 0.1;

                if (Speed < 0) {
                    Speed = 0;
                }
            }

            // 1) Axial:    Forward and backward
            // 2) Lateral:  Right and left
            // 3) Yaw:      Rotating Clockwise and counterclockwise

            // Convert the joystick axis into variables
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // intake control
            if (gamepad1.x) {
                intake.setPosition(intake.getPosition() + 0.2);
            } else {
                intake.setPosition(0);
            }
            // the control for shooting (still in progress)
            if (gamepad1.left_trigger_pressed){
                // TODO: Add shooting mechanism
            }

            // Combine the joystick requests each wheel's power.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Make sure no wheel is over 1.0
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Factor in speed
            frontLeftPower = frontLeftPower * Speed;
            frontRightPower = frontRightPower * Speed;
            backLeftPower = backLeftPower * Speed;
            backRightPower = backRightPower * Speed;

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed Multiplier", Speed);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);

            Pose2D pose2D = OdometryHub.getPosition();

            telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.CM));
            telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.CM));
            telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

    public void InitOdometry() {
        // Distance of Odometry Pods from actual center position
        OdometryHub.setOffsets(0, -0, DistanceUnit.MM);

        // We use SwingArm pods.
        OdometryHub.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        // Set the direction of the X and Y pod.
        // X increase when moving forward. Y (strafe) pod should increase when moving left.

        OdometryHub.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Recalibration position and IMU to start at (0,0)
        OdometryHub.resetPosAndIMU();
    }
    
}
