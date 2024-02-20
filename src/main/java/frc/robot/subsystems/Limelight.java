package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.lib.LimelightHelpers.LimelightResults;
import frc.lib.LimelightHelpers.Results;
import frc.robot.util.Alerts;

public class Limelight extends SubsystemBase {
    
    private Pose2d latestPose;
    private double latestDelay;
    private Translation3d latestDistance;
    
    private boolean validMeasurements;
    private Timer usageTimer = new Timer();

    public Limelight () {

        LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.setLEDMode_PipelineControl("limelight");

        LimelightHelpers.setCropWindow("limelight", -1, 1, -1, 1);

        this.usageTimer.reset();
        this.usageTimer.start();
    }

    @Override
    public void periodic () {

        LimelightResults limelightResults = LimelightHelpers.getLatestResults("limelight");
        Results targetingResults = limelightResults.targetingResults;
        this.validMeasurements = targetingResults.valid;

        if (this.validMeasurements) {

            if (!DriverStation.getAlliance().isPresent()) { this.latestPose = targetingResults.getBotPose2d(); } 
            else if (DriverStation.getAlliance().get() == Alliance.Red) { this.latestPose = targetingResults.getBotPose2d_wpiRed(); } 
            else { this.latestPose = targetingResults.getBotPose2d_wpiBlue(); }

            this.latestDelay = targetingResults.latency_capture + targetingResults.latency_pipeline;
            this.usageTimer.restart();

            Translation3d[] tagDistances = new Translation3d[targetingResults.targets_Fiducials.length];
            for (int i = 0; i < targetingResults.targets_Fiducials.length; i++) { tagDistances[i] = targetingResults.targets_Fiducials[i].getTargetPose_CameraSpace().getTranslation(); }

            double tagDistanceX, tagDistanceY, tagDistanceZ;
            tagDistanceX = tagDistanceY = tagDistanceZ = 0.0;

            for (Translation3d tagDistance : tagDistances) {

                tagDistanceX += tagDistance.getX();
                tagDistanceY += tagDistance.getY();
                tagDistanceZ += tagDistance.getZ();
            }

            tagDistanceX /= tagDistances.length;
            tagDistanceY /= tagDistances.length;
            tagDistanceZ /= tagDistances.length;

            this.latestDistance = new Translation3d(tagDistanceX, tagDistanceY, tagDistanceZ);
        }

        double lastUpdated = this.usageTimer.get();
        SmartDashboard.putString("Limelight Last Updated", (Math.round(lastUpdated * 10) / 10.0) + "s Ago");

        if (lastUpdated >= 20) { Alerts.limelightDetections.set(true); }
        else { Alerts.limelightDetections.set(false); }
    }

    public Pose2d getLatestPose () { return this.latestPose; }
    public double getLatestDelay () { return this.latestDelay; }
    public Translation3d getLatestDistance () { return this.latestDistance; }
    public boolean areValidMeasurements () { return this.validMeasurements; }
}
