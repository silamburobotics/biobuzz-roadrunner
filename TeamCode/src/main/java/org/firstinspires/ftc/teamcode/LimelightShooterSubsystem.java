package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * Reusable Subsystem for Limelight 3A Vision, AprilTag tracking, distance calculation,
 * dynamic flywheel RPM calculation for Pollen & Nectar ball types, and shifter/hood servo angle calculation.
 */
@Config
public class LimelightShooterSubsystem {

    // Configuration Constants
    public static int APRILTAG_PIPELINE_INDEX = 0;
    public static double LIMELIGHT_LENS_HEIGHT_INCHES = 12.0;
    public static double TARGET_CENTER_HEIGHT_INCHES = 36.0;
    public static double LIMELIGHT_MOUNT_ANGLE_DEG = 45.0;     // 45 degree upward camera mounting pitch angle
    public static int DESIRED_TAG_ID = -1;                 // -1 targets any AprilTag

    public static double TURRET_AUTO_AIM_GAIN = 1.0;       // 1.0 = direct 1:1 target angle calculation
    public static double TURRET_AIM_OFFSET_DEG = 0.0;      // Horizontal offset angle (in degrees) from AprilTag center to target basket
    public static double AIM_DEADBAND_DEG = 0.75;          // Deadband tolerance to prevent hunting/oscillations
    public static double TURRET_MIN_ANGLE_DEG = -90.0;
    public static double TURRET_MAX_ANGLE_DEG = 90.0;

    // Ball Type Flywheel RPM Curves (Pollen = Smaller Ball, Nectar = Bigger Ball)
    public static double POLLEN_BASE_RPM = 2400.0;         // Base RPM for Pollen (Smaller ball)
    public static double POLLEN_RPM_SLOPE = 12.0;          // RPM increase per inch of distance
    public static double NECTAR_BASE_RPM = 2400.0;         // Base RPM for Nectar (Bigger ball)
    public static double NECTAR_RPM_SLOPE = 15.0;          // RPM increase per inch of distance

    public static double FLYWHEEL_MIN_RPM = 2000.0;
    public static double FLYWHEEL_MAX_RPM = 4500.0;

    public static double SHIFTER_OPEN_POSITION = 0.25;
    public static double SHIFTER_CLOSED_POSITION = 0.95;

    // Alliance AprilTag Filtering Constants
    // Blue Alliance Tags: Left (39, 40), Right (43, 44)
    // Red Alliance Tags: Left (31, 32), Right (35, 36)
    public static final int[] BLUE_TAG_IDS = {39, 40, 43, 44};
    public static final int[] RED_TAG_IDS = {31, 32, 35, 36};

    // Hardware
    private Limelight3A limelight;

    // State Variables
    private boolean targetVisible = false;
    private double tx = 0.0;
    private double ty = 0.0;
    private double calculatedDistanceInches = 0.0;
    private int detectedTagId = -1;

    // Target Lock-On State
    private int lockedTagId = -1;
    private boolean targetLocked = false;
    private int framesSinceTagSeen = 0;
    private static final int MAX_PERSISTENCE_FRAMES = 10; // Persistence during brief vision occlusion (~300ms)

    public LimelightShooterSubsystem(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception ignored) {
            limelight = null;
        }

        if (limelight == null) {
            List<Limelight3A> limelights = hardwareMap.getAll(Limelight3A.class);
            if (limelights != null && !limelights.isEmpty()) {
                limelight = limelights.get(0);
            }
        }

        if (limelight != null) {
            limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
            limelight.start();
        }
    }

    /**
     * Checks if a given AprilTag ID belongs to the specified alliance.
     * Blue Alliance Tags: Left (39, 40), Right (43, 44)
     * Red Alliance Tags: Left (31, 32), Right (35, 36)
     */
    public static boolean isAllianceTag(int tagId, boolean isRedAlliance) {
        if (isRedAlliance) {
            return tagId == 31 || tagId == 32 || tagId == 35 || tagId == 36;
        } else {
            return tagId == 39 || tagId == 40 || tagId == 43 || tagId == 44;
        }
    }

    /**
     * Resets the target lock-on state to allow acquiring a new AprilTag.
     */
    public void resetLock() {
        lockedTagId = -1;
        targetLocked = false;
        targetVisible = false;
        framesSinceTagSeen = 0;
    }

    /**
     * Polls the Limelight 3A for the latest frame results and updates vision metrics
     * filtered strictly by the active alliance tag set.
     * Automatically locks onto detected target tags and maintains lock stability.
     */
    public void update(boolean isRedAlliance) {
        LLResult result = (limelight != null) ? limelight.getLatestResult() : null;

        targetVisible = false;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                LLResultTypes.FiducialResult matchedFiducial = null;

                // 1. If currently locked on a tag, check if lockedTagId is still visible and valid for current alliance
                if (lockedTagId != -1 && isAllianceTag(lockedTagId, isRedAlliance)) {
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == lockedTagId) {
                            matchedFiducial = fiducial;
                            break;
                        }
                    }
                }

                // 2. If locked tag was not found in view (or not locked yet), acquire best valid alliance tag
                if (matchedFiducial == null) {
                    double bestTx = Double.MAX_VALUE;
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        int fid = fiducial.getFiducialId();
                        if (isAllianceTag(fid, isRedAlliance)) {
                            double absTx = Math.abs(fiducial.getTargetXDegrees());
                            if (absTx < bestTx) {
                                bestTx = absTx;
                                matchedFiducial = fiducial;
                            }
                        }
                    }
                }

                // 3. Process matched valid alliance tag
                if (matchedFiducial != null) {
                    targetVisible = true;
                    targetLocked = true;
                    lockedTagId = matchedFiducial.getFiducialId();
                    detectedTagId = lockedTagId;
                    tx = matchedFiducial.getTargetXDegrees();
                    ty = matchedFiducial.getTargetYDegrees();
                    framesSinceTagSeen = 0;

                    double angleToTargetRad = Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEG + ty);
                    double heightDifference = TARGET_CENTER_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES;

                    if (Math.tan(angleToTargetRad) > 0) {
                        double rawDistance = heightDifference / Math.tan(angleToTargetRad);
                        calculatedDistanceInches = Range.clip(rawDistance, 12.0, 144.0);
                    }
                }
            }
        }

        // 4. Persistence handling if tag is briefly lost in current frame
        if (!targetVisible) {
            if (targetLocked && framesSinceTagSeen < MAX_PERSISTENCE_FRAMES) {
                framesSinceTagSeen++;
                targetVisible = true; // Retain target lock and distance during brief occlusion
            } else {
                targetLocked = false;
                lockedTagId = -1;
                detectedTagId = -1;
                tx = 0.0;
                ty = 0.0;
                framesSinceTagSeen = 0;
            }
        }
    }

    /**
     * Default update method (defaults to Blue Alliance).
     */
    public void update() {
        update(false);
    }

    public boolean isLimelightConnected() {
        return limelight != null;
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }

    public boolean isTargetLocked() {
        return targetLocked && targetVisible;
    }

    public int getLockedTagId() {
        return lockedTagId;
    }

    public boolean isAimed() {
        return targetVisible && Math.abs(getTxError()) <= AIM_DEADBAND_DEG;
    }

    public double getTx() {
        return tx;
    }

    public double getTxError() {
        return tx - TURRET_AIM_OFFSET_DEG;
    }

    public double getTy() {
        return ty;
    }

    public double getCalculatedDistanceInches() {
        return calculatedDistanceInches;
    }

    public int getDetectedTagId() {
        return detectedTagId;
    }

    /**
     * Calculates the new target angle for the turret to align with the target shooting area,
     * offset relative to the AprilTag center, constrained to soft limits [-90°, +90°].
     */
    public double getTurretTargetAngle(double currentTurretAngleDeg) {
        double errorTx = getTxError();
        if (!targetVisible || Math.abs(errorTx) <= AIM_DEADBAND_DEG) {
            return currentTurretAngleDeg;
        }
        double desiredTurretAngle = currentTurretAngleDeg - (errorTx * TURRET_AUTO_AIM_GAIN);
        return Range.clip(desiredTurretAngle, TURRET_MIN_ANGLE_DEG, TURRET_MAX_ANGLE_DEG);
    }

    /**
     * Calculates required flywheel RPM dynamically based on AprilTag distance and ball type:
     * - isNectarMode = false (Pollen / Smaller Ball)
     * - isNectarMode = true (Nectar / Bigger Ball)
     */
    public double getFlywheelTargetRPM(boolean isNectarMode) {
        double baseRPM = isNectarMode ? NECTAR_BASE_RPM : POLLEN_BASE_RPM;
        if (!targetVisible || calculatedDistanceInches <= 0) {
            return baseRPM;
        }
        double slope = isNectarMode ? NECTAR_RPM_SLOPE : POLLEN_RPM_SLOPE;
        double calculatedRPM = baseRPM + (slope * calculatedDistanceInches);
        return Range.clip(calculatedRPM, FLYWHEEL_MIN_RPM, FLYWHEEL_MAX_RPM);
    }

    /**
     * Calculates required shifter/hood servo position dynamically based on distance to the AprilTag.
     */
    public double getShifterTargetPosition() {
        if (!targetVisible || calculatedDistanceInches <= 0) return SHIFTER_CLOSED_POSITION;
        double minDistance = 24.0;
        double maxDistance = 120.0;
        double normDistance = Range.clip((calculatedDistanceInches - minDistance) / (maxDistance - minDistance), 0.0, 1.0);
        return SHIFTER_CLOSED_POSITION - normDistance * (SHIFTER_CLOSED_POSITION - SHIFTER_OPEN_POSITION);
    }

    /**
     * Stops Limelight polling on OpMode shutdown.
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
