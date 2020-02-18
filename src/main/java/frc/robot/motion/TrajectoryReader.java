package frc.robot.motion;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectReader;

import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryReader {
    private static Trajectory[] trajectories;

    public static void loadTrajectories() throws IOException {
        loadTrajectories(Filesystem.getDeployDirectory().toPath().resolve("trajectories.json"));
    }

    protected static void loadTrajectories(Path filePath) throws IOException {
        if (trajectories != null)
            return;
        BufferedReader reader;
        reader = Files.newBufferedReader(filePath);
        ObjectReader READER = new ObjectMapper().readerFor(Trajectory[].class);
        trajectories = READER.readValue(reader);

    }

    public static Trajectory getTrajectory(String trajectoryName) throws IOException {
        loadTrajectories();
        for (Trajectory trajectory : trajectories) {
            if (trajectory.name.equals(trajectoryName)) {
                return trajectory;
            }
        }
        throw new RuntimeException("Could not find trajectory by name: " + trajectoryName);
    }
}