// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class YawCalibration extends Command {
  private PhotonPoseEstimator photonPoseEstimator;
  private Limelight limelight;
  private CommandSwerveDrivetrain driveTrain;
  private ArrayList<double[]> input = new ArrayList<>();
  private final Transform3d robotToCamera = new Transform3d(0.287, 0.275, 0.395, new Rotation3d(3.0042,0.2186,0));
  /** Creates a new YawCalibration. */
  public YawCalibration(CommandSwerveDrivetrain driveTrain, Limelight limelight) {
      this.driveTrain = driveTrain;
      AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
      photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
      this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      List<PhotonPipelineResult> results = limelight.getAllUnreadResults();
      for (PhotonPipelineResult result : results) {
        boolean inRange = false;
        double highestAmbiguity = 0;
        for (PhotonTrackedTarget target: result.getTargets()) {
            double xSquared = target.getBestCameraToTarget().getX() * target.getBestCameraToTarget().getX();
            double ySquared = target.getBestCameraToTarget().getY() * target.getBestCameraToTarget().getY();
            double distance = Math.sqrt(xSquared + ySquared);
            if(target.getPoseAmbiguity() > highestAmbiguity) {
                highestAmbiguity = target.getPoseAmbiguity();
            }
            if(distance < 2) {
                inRange = true;
            } else {
                inRange = false;
            }
            //scaledVisionStdDeviations = visionStdDeviations.times(1 + distance);
        }
        if(DriverStation.isDSAttached() && DriverStation.isDisabled()) {
            inRange = true;
        }

        Optional<EstimatedRobotPose> optionalOdoState = photonPoseEstimator.update(result);
        if (optionalOdoState.isPresent()) {
            EstimatedRobotPose odoState = optionalOdoState.get();
            photonPoseEstimator.setReferencePose(driveTrain.getState().Pose);
            Pose2d estimatedPose = odoState.estimatedPose.toPose2d();
            double[] poseDouble = {estimatedPose.getX(), estimatedPose.getY()};
            input.add(poseDouble);
        }
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 if (input.isEmpty()) {
            System.out.println("Calibration failed: No samples.");
            return;
        }

        double[][] X = new double[input.size()][3];
        double[] Z = new double[input.size()];
        for (int i = 0; i < input.size(); i++) {
            double[] p = input.get(i);
            X[i][0] = p[0];
            X[i][1] = 1.0;
            Z[i] = p[1]; 
        }

        OLSMultipleLinearRegression regression = new OLSMultipleLinearRegression();
        regression.newSampleData(Z, X);

        double[] params = regression.estimateRegressionParameters();
        double a = params[0];
        double c = params[1];

        double yaw = -Math.atan(a);

        System.out.println("RESULT");
        System.out.println("Slope = " +  a + "\n Yaw = " + yaw);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
