package frc.robot.commands.Camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class HandEyeCalibration extends Command {
    private final Camera camera;
    private final CommandSwerveDrivetrain drivetrain;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    private final List<Transform3d> calculatedRobotToCameras = new ArrayList<>();

    private static final double maxVelocity = 0.05;
    private static final double maxAngularVelocity = 0.05;

    public HandEyeCalibration(Camera camera, CommandSwerveDrivetrain drivetrain) {
        this.camera = camera;
        this.drivetrain = drivetrain;

        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        this.aprilTagFieldLayout = layout;
    }

    @Override
    public void initialize() {
        calculatedRobotToCameras.clear();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = drivetrain.getState().Speeds;
        boolean isStable = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < maxVelocity
                && Math.abs(speeds.omegaRadiansPerSecond) < maxAngularVelocity;

        if (!isStable) return;

        PhotonPipelineResult result = camera.getLatestResult();
        Pose2d robotPose2d = drivetrain.getState().Pose;
        Pose3d robotPose3d = new Pose3d(robotPose2d);

        if (!result.hasTargets()) return;

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getPoseAmbiguity() > 0.2) continue;

            int id = target.getFiducialId();
            Optional<Pose3d> tagOnField = aprilTagFieldLayout.getTagPose(id);

            if (tagOnField.isPresent()) {
                Transform3d fieldToRobot = new Transform3d(robotPose3d.getTranslation(), robotPose3d.getRotation());
                Transform3d fieldToTag = new Transform3d(tagOnField.get().getTranslation(), tagOnField.get().getRotation());

                Transform3d cameraToTag = target.getBestCameraToTarget();

                Transform3d robotToCamera = fieldToRobot.inverse()
                        .plus(fieldToTag)
                        .plus(cameraToTag.inverse());

                calculatedRobotToCameras.add(robotToCamera);
                System.out.println("Measurement taken. Total: " + calculatedRobotToCameras.size());
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (calculatedRobotToCameras.isEmpty()) {
            System.out.println("Calibration failed: No samples.");
            return;
        }

        double sumX = 0, sumY = 0, sumZ = 0;
        double sumQw = 0, sumQx = 0, sumQy = 0, sumQz = 0;

        Quaternion refQ = calculatedRobotToCameras.get(0).getRotation().getQuaternion();

        for (Transform3d t : calculatedRobotToCameras) {
            sumX += t.getX();
            sumY += t.getY();
            sumZ += t.getZ();

            Quaternion q = t.getRotation().getQuaternion();

            if (q.dot(refQ) < 0) {
                q = new Quaternion(-q.getW(), -q.getX(), -q.getY(), -q.getZ());
                q = q.normalize();
            }

            sumQw += q.getW();
            sumQx += q.getX();
            sumQy += q.getY();
            sumQz += q.getZ();
        }

        int n = calculatedRobotToCameras.size();
        Quaternion avgQ = new Quaternion(sumQw/n, sumQx/n, sumQy/n, sumQz/n).normalize();

        System.out.println("RESULT");
        System.out.println("Transform3d robotToCamera = " + sumX/n + ", " + sumY/n + ", " + sumZ/n);
        System.out.println("Rotation 3d robotToCamera from Quaternion = " + avgQ.getW() + ", " + avgQ.getX() + ", " + avgQ.getY() + ", " + avgQ.getZ());
    }
}