package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.geometry.Pose;

/**
 * Singleton class to hold the robot's current position on the field.
 * Can be accessed from any class to get the robot's pose.
 */
public class RobotState {
    private static RobotState instance;

    private Pose currentPose = new Pose(0, 0, 0);
    private boolean isBlueAlliance = true;

    private RobotState() {}

    public static synchronized RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    public void setPose(Pose pose) {
        this.currentPose = pose;
    }

    public Pose getPose() {
        return currentPose;
    }

    public void setAlliance(boolean isBlue) {
        this.isBlueAlliance = isBlue;
    }

    public boolean isBlueAlliance() {
        return isBlueAlliance;
    }

    /**
     * Calculate distance from robot to the appropriate goal based on alliance
     */
    public double getDistanceToGoal() {
        Pose goalPose = isBlueAlliance ? ShooterConstraints.GOAL_POSE_BLUE : ShooterConstraints.GOAL_POSE_RED;
        double dx = currentPose.getX() - goalPose.getX();
        double dy = currentPose.getY() - goalPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Get angle from robot to goal (for potential turret aiming)
     */
    public double getAngleToGoal() {
        Pose goalPose = isBlueAlliance ? ShooterConstraints.GOAL_POSE_BLUE : ShooterConstraints.GOAL_POSE_RED;
        double dx = goalPose.getX() - currentPose.getX();
        double dy = goalPose.getY() - currentPose.getY();
        return Math.atan2(dy, dx);
    }
}

