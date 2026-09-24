/* Copyright (c) 2023 FIRST. All rights reserved.
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

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

@TeleOp(name = "AprilTagTest", group = "Tests")
public class AprilTagTest extends LinearOpMode {

    // Hardware \\
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private Servo rightServo = null;
    private ServoController servoController;

    // Camera Variables \\
    private VisionPortal visionPortal; // Our instance of the vision portal.

    // AprilTag Variables \\
    private AprilTagProcessor aprilTag; // Our instance of the AprilTag processor.
    private int aprilTagTargetID = -1; // The tag to target, or -1 for any.

    // ColorBlob Variables \\
    PredominantColorProcessor colorSensor;


    // Detection Variables \\
    private boolean targetFound  = false;
    private String targetName    = "none";
    private int    targetID      = 0;
    private double targetRange   = 0;
    private double targetBearing = 0;
    private double targetYaw     = 0;



    private int ranAmount = 0;

    // OpMode \\
    @Override
    public void runOpMode() {

        // Get the configured motors.
        leftMotor  = hardwareMap.get(DcMotor.class, "LeftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "RightMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Get the configured servo.
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        rightServo.scaleRange(0.0, 1.0);
        rightServo.setPosition(0.5);
        rightServo.setDirection(Servo.Direction.FORWARD);

        servoController = rightServo.getController();
        servoController.pwmEnable();

        initCamera(); // Initialize the vision portal as well as the color blob and april tag sensors.

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart(); // Wait until the OpMode is started.

        while (opModeIsActive()) {
            // Reset the found AprilTag
            targetFound = false;


            ranAmount += 1;
            telemetry.addData("Ran", "Ran " + ranAmount + " Times.");

            /* This code is commented as it currently isn't in use.
            telemetryAprilTag();
            telemetry.update();
            */


            // Gamepad Controls \\

            if (gamepad1.dpad_down) { // When dpad down is pressed, turn off streaming, save resources
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) { // When dpad up is pressed, turn on streaming
                visionPortal.resumeStreaming();
            }

            if (gamepad1.dpad_left) {
                // When dpad left is pressed, decrease AprilTag target by 1
                aprilTagTargetID -= 1;

                if (aprilTagTargetID < 0) {
                    telemetry.addData("Target April Tag", aprilTagTargetID + " = Any AprilTag");
                } else {
                    telemetry.addData("Target April Tag", aprilTagTargetID);
                }
            } else if (gamepad1.dpad_right) {
                // When dpad right is pressed, increase AprilTag target by 1
                aprilTagTargetID += 1;

                if (aprilTagTargetID < 0) {
                    telemetry.addData("Target April Tag", "Any AprilTag");
                } else {
                    telemetry.addData("Target April Tag", aprilTagTargetID);
                }
            }


            // AprilTag Detections \\

            List<AprilTagDetection> currentDetections = aprilTag.getDetections(); // Get all detected april tags

            for (AprilTagDetection detection : currentDetections) { // For each detection in the detected april tags...

                // If the detection is a Single AprilTag...
                if (detection instanceof AprilTagSingleDetection) {

                    // Change the detection type from an `AprilTagDetection` to a `AprilTagSingleDetection`
                    AprilTagSingleDetection singleDetection = (AprilTagSingleDetection) detection;

                    // Check if there is information on this tag in SDK Library
                    if (singleDetection.metadata != null) {
                        // Check if the tag is the target
                        if (aprilTagTargetID < 0 || singleDetection.id == aprilTagTargetID) {
                            // If it's the target, lets get information from it.
                            targetName    = singleDetection.metadata.name;
                            targetID      = singleDetection.id;
                            targetRange   = singleDetection.ftcPose.range;
                            targetBearing = singleDetection.ftcPose.bearing;
                            targetYaw     = singleDetection.ftcPose.yaw;
                            targetFound   = true;
                            break; // Take the first applicable tag, don't scan any others.
                        } else {
                            // This tag is in the library but is not a target.
                            telemetry.addData("Skipping Tag", "Tag ID " + singleDetection.id + " is not desired");
                        }

                    } else {
                        telemetry.addData("Unknown Tag", "Tag is not in SDK Library");
                    }

                } else {
                    // TODO: Account for tag clusters.
                    telemetry.addData("Wrong Tag Type", "Tag is not a SingleTag. It may be a cluster or other type.");
                }

            }

            if (targetFound) {
                /*
                Move the Servo.

                Servo Values are 0-1, with .5 at the center.
                targetBearing Values are -180 (180 degrees left) to 180 (180 degrees right), with 0 at the center.
                */
                double currentPos = rightServo.getPosition();
                telemetry.addData("currentPos", currentPos);

                double targetPos  = 0.5 + (targetBearing / 180.0) * .5; // Translate the bearing to a servoPosition
                telemetry.addData("targetPos", targetPos);

                double servoDelta = targetPos - currentPos;

                // Set the new position of the servo, but ease it so it doesn't snap around abruptly.
                rightServo.setPosition(currentPos + servoDelta * 0.1);


                // Telemetry the AprilTag Data
                telemetry.addData("Found", "ID %d (%s)", targetID, targetName);
                telemetry.addData("Range",  "%5.1f inches", targetRange);
                telemetry.addData("Bearing","%3.0f degrees", targetBearing);
                telemetry.addData("Yaw","%3.0f degrees", targetYaw);
            } else { // If there isn't an AprilTag, send ColorBlob information instead.
                PredominantColorProcessor.Result result = colorSensor.getAnalysis();
                telemetry.addData("Best Match", result.closestSwatch);
                // Formatting returns 3 integers up to 3 digits for Red, Green, and Blue.
                telemetry.addLine(String.format( "RGB = (%3d, %3d, %3d)", result.RGB[0], result.RGB[1], result.RGB[2]));
            }

            telemetry.update();

            // Check only 50 times a second and give the CPU a break.
            sleep(20);
        }

        // Save resources once the OpMode has ended.
        visionPortal.close();
    }

    private void initCamera() {

        aprilTag = new AprilTagProcessor.Builder()

                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setDrawCubeProjection(false)

                .build();
        // end

        colorSensor = new PredominantColorProcessor.Builder()
                // The Region of Interest to check the color of.
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                // All the preset colors to look for.
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();
        // end

        VisionPortal.Builder builder = new VisionPortal.Builder();

            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new Size(640, 480));
            builder.enableLiveView(true); // Enable live preview of camera
            builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
            builder.setAutoStopLiveView(false);

            // Set and enable the processors.
            builder.addProcessor(aprilTag);
            builder.addProcessor(colorSensor);

            // Build the Vision Portal, using the above settings.
            visionPortal = builder.build();
        // end

    }

}
