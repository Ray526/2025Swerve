package frc.robot.commandFactory;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PathplannerFactory {

    public static Command driveToSetpointCommand(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
                3,
                3,
                2 * Math.PI,
                4 * Math.PI);

        return AutoBuilder.pathfindToPose(targetPose, constraints, 0);
    }

    public static Command driveThenFollowPath(String pathName) throws FileVersionException, IOException, ParseException {
        PathPlannerPath path;
        
        PathConstraints constraints = new PathConstraints(
                3,
                3,
                2 * Math.PI,
                4 * Math.PI);

        path = PathPlannerPath.fromPathFile(pathName);

        return AutoBuilder.pathfindThenFollowPath(path, constraints);
    }
}