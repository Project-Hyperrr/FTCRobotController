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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * the easy way.
 *
 * Note: See ConceptAprilTag.java for how to add a camera compatibility quirk
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 */
@TeleOp(name = "Concept: AprilTag Easy", group = "Concept")
public class AprilTagTest extends LinearOpMode {

    // Hardware \\
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private Servo rightServo = null;

    // AprilTag Variables \\
    private static final boolean USE_WEBCAM = true;  // We're using a webcam so this is true.

    private AprilTagProcessor aprilTag; // Our instance of the AprilTag processor.
    private VisionPortal visionPortal; // Our instance of the vision portal.

    private int aprilTagTargetID = -1; // The tag to target, or -1 for any.

    // Detection Variables ||
    private boolean targetFound  = false;
    private String targetName    = "none";
    private int    targetID      = 0;
    private double targetRange   = 0;
    private double targetBearing = 0;
    private double targetYaw     = 0;

    // OpMode \\
    @Override
    public void runOpMode() {

        // Get the configured hardware
        leftMotor  = hardwareMap.get(DcMotor.class, "LeftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "RightMotor");

        rightServo = hardwareMap.get(Servo.class, "rightServo");

        // Flip the motors on one side so they all move the same direction
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // See ConceptAprilTag.java for how to add a camera compatibility quirk
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart(); // Wait until the OpMode is started.

        while (opModeIsActive()) {
            // Reset the found AprilTag
            targetFound = false;

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
                telemetry.update();
            } else if (gamepad1.dpad_right) {
                // When dpad right is pressed, increase AprilTag target by 1
                aprilTagTargetID += 1;

                if (aprilTagTargetID < 0) {
                    telemetry.addData("Target April Tag", "Any AprilTag");
                } else {
                    telemetry.addData("Target April Tag", aprilTagTargetID);
                }
                telemetry.update();
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
                            telemetry.update();
                        }

                    } else {
                        telemetry.addData("Unknown Tag", "Tag is not in SDK Library");
                        telemetry.update();
                    }

                } else {
                    // TODO: Account for tag clusters.
                    telemetry.addData("Wrong Tag Type", "Tag is not a SingleTag. It may be a cluster or other type.");
                    telemetry.update();
                }

            }

            if (targetFound) {
                /*
                Move the Servo.

                Servo Values are 0-1, with .5 at the center.
                targetBearing Values are -180 (180 degrees left) to 180 (180 degrees right), with 0 at the center.
                */
                double currentPos = rightServo.getPosition();
                double targetPos  = 0.5 + (targetBearing / 180.0) * .5; // Translate the bearing to a servoPosition
                double servoDelta = targetPos - currentPos;

                // Set the new position of the servo, but ease it so it doesn't snap around abruptly.
                rightServo.setPosition(currentPos + servoDelta * 0.1);

                // Telemetry the AprilTag Data
                telemetry.addData("Found", "ID %d (%s)", targetID, targetName);
                telemetry.addData("Range",  "%5.1f inches", targetRange);
                telemetry.addData("Bearing","%3.0f degrees", targetBearing);
                telemetry.addData("Yaw","%3.0f degrees", targetYaw);
            }

            // Check only 50 times a second and give the CPU a break.
            sleep(20);
        }

        // Save resources once the OpMode has ended.
        visionPortal.close();

    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                BuiltinCameraDirection.BACK, aprilTag);
        }

    }

    // The following code is commented out because I don't believe it's currently necessary.

    /*
    // Add telemetry about AprilTag detections.
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {

            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                }

            }  else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
    */
}   // end class
