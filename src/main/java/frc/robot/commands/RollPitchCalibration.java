package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class RollPitchCalibration extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    public ArrayList<double[]> input = new ArrayList<>();

    public double fiducialID = 19;

    public RollPitchCalibration(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        input.clear();
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = new PhotonPipelineResult();

        if (!result.hasTargets()) return;

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getPoseAmbiguity() > 0.2) continue;

            if (target.getFiducialId() == fiducialID) {
                Transform3d cameraToTag = target.getBestCameraToTarget();
                double[] point = {cameraToTag.getX(), cameraToTag.getY(), cameraToTag.getZ()};
                input.add(point);

                System.out.println("Measurement taken. Total: " + input.size());
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

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
            X[i][1] = p[1];
            X[i][2] = 1.0;
            Z[i] = p[2]; 
        }

        OLSMultipleLinearRegression regression = new OLSMultipleLinearRegression();
        regression.newSampleData(Z, X);

        double[] params = regression.estimateRegressionParameters();
        double a = params[0];
        double b = params[1];
        double d = params[2];

        double[] normal = new double[]{a, b, -1};
        double mag = Math.sqrt(a*a + b*b + 1);
        normal[0] /= mag;
        normal[1] /= mag;
        normal[2] /= mag;

        double roll = Math.atan2(normal[1], normal[2]);
        double pitch = -Math.atan2(normal[0], normal[2]);

        System.out.println("RESULT");
        System.out.println("Roll = " +  roll + "\n Pitch = " + pitch);
    }
}