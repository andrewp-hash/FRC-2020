package frc.robot.motion;

import java.io.IOException;

class Trajectory {
    public String name;
    public TrajectoryPoint[] points;

    static Trajectory fromJSON(String trajectoryName) {
        try {
            return TrajectoryReader.getTrajectory(trajectoryName);
        } catch (IOException e) {
            System.err.println("Unable to load trajectory: " + trajectoryName);
        }
        return null;
    }
}