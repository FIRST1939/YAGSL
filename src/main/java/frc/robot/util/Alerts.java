package frc.robot.util;

import frc.lib.Alert;
import frc.lib.Alert.AlertType;

public final class Alerts {
    
    public static final Alert versionControl = new Alert("Current Code Not Under Version Control", AlertType.WARNING);
    public static final Alert lowBattery = new Alert("Battery Voltage Low", AlertType.WARNING);

    public static final Alert swerveInitialized = new Alert("Swerve Configuration Failed to Initialize", AlertType.ERROR);
    public static final Alert limelightDetections = new Alert("Limelight Hasn't Detected AprilTag in >20s", AlertType.WARNING);
    public static final Alert driverOneDisconnected = new Alert("Driver One Controller Disconnected", AlertType.ERROR);
}
