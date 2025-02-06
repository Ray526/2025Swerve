package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.commandFactory.PathplannerFactory;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestSwerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.Vision;
import frc.robot.subsystems.Oiiai;

public class RobotContainer {

    private final Oiiai oiiai = new Oiiai();
    public final Swerve swerve = Swerve.getInstance();
    public final Vision vision = new Vision();
    private final Elevator elevator = new Elevator();

    private final CommandXboxController driveController = new CommandXboxController(0);

    private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, driveController);
    // private final TestSwerve testSwerve = new TestSwerve(swerve);
    // private final Command driveToOneOne = PathplannerFactory
    //         .driveToSetpointCommand(new Pose2d(1.5, 4.5, Rotation2d.fromDegrees(180)));
    // private final Command driveToOneSix = PathplannerFactory
    //         .driveToSetpointCommand(new Pose2d(3.5, 4.5, Rotation2d.fromDegrees(0)));
    // private Command driveToFollowPathLeft = null;

    private final SendableChooser<Command> autoChooser;
    public Field2d m_field = new Field2d();

    private Timer timer = new Timer();

    private Pose2d currentPose = new Pose2d();
    private Rotation2d currentAngle = new Rotation2d();
    private Pose2d lastPose = new Pose2d();
    private Rotation2d lastAngle = new Rotation2d();
    private Translation2d poseError = new Translation2d();
    private Rotation2d rotationError = new Rotation2d();

    private boolean isAdjusted = false;

    public RobotContainer() {

        // try {
        //     driveToFollowPathLeft = PathplannerFactory.driveThenFollowPath("Left");
        // } catch (Exception e) {
        //     driveToFollowPathLeft = Commands.none();
        //     System.out.println("driveToFollowPathLeft is a none command");
        // }

        swerve.setDefaultCommand(teleopSwerve);
        swerve.setOdometryPosition(FieldConstants.A);

        configureButtonBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("choose auto",null);
        autoChooser.addOption("1M", new PathPlannerAuto("1M"));
        autoChooser.addOption("Turn", new PathPlannerAuto("Turn"));
        autoChooser.addOption("FullAuto", new PathPlannerAuto("FullAuto"));
        autoChooser.addOption("CSR-C", new PathPlannerAuto("CSR-C"));
        autoChooser.addOption("CSL-A", new PathPlannerAuto("CSL-A"));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void addVision() {
        /* ========================= VISION ========================= */
        // Correct pose estimate with vision measurements
        var visionEst = vision.getEstimatedGlobalPose();
        visionEst.ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = vision.getEstimationStdDevs();

                    swerve.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
    }

    private BooleanSupplier isDriverControl = () -> {
        return driveController.getLeftX() != 0 || driveController.getLeftY() != 0;
    };

    private void configureButtonBindings() {
        // driveController.x().toggleOnTrue(driveToOneOne.until(isDriverControl));
        // driveController.y().toggleOnTrue(driveToOneSix.until(isDriverControl));
        // driveController.b().toggleOnTrue(driveToFollowPathLeft.until(isDriverControl));
    }

    public boolean odometryCloseToPose(Pose2d pose) {
        Pose2d fieldToRobot = vision.getEstimatedGlobalPose().isPresent() ? vision.getEstimatedGlobalPose().get().estimatedPose.toPose2d() : swerve.getEstimatedPosition();
        double distance = fieldToRobot.getTranslation().getDistance(pose.getTranslation());
        m_field.setRobotPose(fieldToRobot);
        SmartDashboard.putNumber("Distance From Start Pose", distance);
        double rotation = Math.abs(fieldToRobot.getRotation().rotateBy(pose.getRotation().unaryMinus()).getDegrees());
        SmartDashboard.putNumber("Rotation From Start Pose", rotation);
        if (distance < 0.25 && rotation < 8.0) {
            return true;
        }
        return false;
    }

    public Command getAutonomousCommand() {
        return (autoChooser.getSelected() == null) ? new PathPlannerAuto("FullAuto") : autoChooser.getSelected();

    }

    public Command getDisabledCommand() {
        // return new ParallelCommandGroup(
        //         // LedFactory.batteryLEDs(this, () -> powerDistribution.getVoltage())
        //     null, null
        // // Add any other subsystem coast commands here
        // ).withName("Disabled command group");
        return swerve.safetystop();
    }

    public void setRumble() {
        timer.restart();
        Command rumbleCommand = Commands.startEnd(
                () -> {
                    driveController.setRumble(RumbleType.kBothRumble, 1);
                },
                () -> {
                    driveController.setRumble(RumbleType.kBothRumble, 0);
                }
                , elevator)
                .until(
                        () -> {
                            return timer.get() > 0.5;
                        });
        rumbleCommand.schedule();
    }

    public void fixingPose() {
    // m_Swerve.Rodometry.update(m_Swerve.getGyroYaw(), m_Swerve.getModulePositions());
    // currentPose = m_Swerve.Rodometry.getPoseMeters();
    // currentPose = swerve.getOdometryPosition();
    // currentAngle = swerve.getGyroYaw();

    if (isDriverControl.getAsBoolean()) {
      // record pose while driver controling
      lastPose = swerve.getEstimatedPosition();
      lastAngle = swerve.getGyroYaw();
      isAdjusted = false;
    }

    poseError = currentPose.minus(lastPose).getTranslation();
    rotationError = currentAngle.minus(lastAngle);

    if (poseError.getX() > 0.1 && poseError.getY() > 0.1 && rotationError.getDegrees() > 3
        && !isAdjusted && !isDriverControl.getAsBoolean()) {
          SmartDashboard.putNumber("poseError X", poseError.getX());
          SmartDashboard.putNumber("psoeError Y", poseError.getY());
          AutoBuilder.pathfindToPose(
            new Pose2d(poseError.plus(currentPose.getTranslation()), 
              rotationError.plus(currentAngle)), 
            new PathConstraints(4.0, 2.0, 
            Units.degreesToRadians(360), Units.degreesToRadians(540)));
          isAdjusted = true;
    }
    }
}
