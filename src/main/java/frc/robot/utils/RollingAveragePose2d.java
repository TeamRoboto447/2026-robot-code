package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.LinkedList;
import java.util.Queue;

public class RollingAveragePose2d {
  private final int windowSize;
  private final Queue<Pose2d> poses;

  private double sumX;
  private double sumY;
  // Instead of summing the "theta" in radians,
  // sum the cos and sin of theta.
  private double sumCos;
  private double sumSin;

  public RollingAveragePose2d(int windowSize) {
    this.windowSize = windowSize;
    this.poses = new LinkedList<>();

    sumX = 0.0;
    sumY = 0.0;
    sumCos = 0.0;
    sumSin = 0.0;
  }

  public void addPose(Pose2d pose) {
    // Add the new pose
    poses.add(pose);
    sumX += pose.getX();
    sumY += pose.getY();
    sumCos += pose.getRotation().getCos();
    sumSin += pose.getRotation().getSin();

    // If we exceed the window size, remove the oldest pose
    if (poses.size() > windowSize) {
      Pose2d removed = poses.poll();
      sumX -= removed.getX();
      sumY -= removed.getY();
      sumCos -= removed.getRotation().getCos();
      sumSin -= removed.getRotation().getSin();
    }
  }

  public Pose2d getAveragePose() {
    if (poses.isEmpty()) {
      return new Pose2d(); // Default to zero pose
    }
    int size = poses.size();

    double avgX = sumX / size;
    double avgY = sumY / size;

    // Compute average rotation via atan2 of the summed cos/sin.
    double avgCos = sumCos / size;
    double avgSin = sumSin / size;
    double avgTheta = Math.atan2(avgSin, avgCos);

    return new Pose2d(avgX, avgY, new Rotation2d(avgTheta));
  }

  public void reset() {
    poses.clear();
    sumX = 0.0;
    sumY = 0.0;
    sumCos = 0.0;
    sumSin = 0.0;
  }
}
