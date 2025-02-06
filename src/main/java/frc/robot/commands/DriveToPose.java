package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib2025.util.MathHelpers;
import frc.robot.subsystems.Swerve;

/**
 * Drives to a specified pose.
 */
public class DriveToPose extends Command {
    private final ProfiledPIDController driveController = new ProfiledPIDController(
            6.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private Swerve swerve;
    private Supplier<Pose2d> poseSupplier;
    private Translation2d lastSetpointTranslation;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double ffMinRadius = 0.2, ffMaxRadius = 0.8;

    public DriveToPose(Swerve swerve, Supplier<Pose2d> poseSupplier) {
        this.swerve = swerve;
        this.poseSupplier = poseSupplier;
        addRequirements(swerve);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerve.getEstimatedPosition();
        driveController.reset(
                currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
                Math.min(
                        0.0,
                        -new Translation2d(swerve.getRobotRelativSpeeds().vxMetersPerSecond,
                            swerve.getRobotRelativSpeeds().vyMetersPerSecond)
                                .rotateBy(
                                        poseSupplier
                                                .get()
                                                .getTranslation()
                                                .minus(swerve.getEstimatedPosition().getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(currentPose.getRotation().getRadians(),
                swerve.getRobotRelativSpeeds().omegaRadiansPerSecond);
        lastSetpointTranslation = swerve.getEstimatedPosition().getTranslation();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getEstimatedPosition();
        Pose2d targetPose = poseSupplier.get();

        Logger.recordOutput("DriveToPose/currentPose", currentPose);
        Logger.recordOutput("DriveToPose/targetPose", targetPose);

        double currentDistance = currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
        double ffScaler = MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);
        double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance())
            driveVelocityScalar = 0.0;
        lastSetpointTranslation = new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(
                        MathHelpers.transform2dFromTranslation(
                                new Translation2d(driveController.getSetpoint().position, 0.0)))
                .getTranslation();

        // Calculate theta speed
        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance())
            thetaVelocity = 0.0;

        // Command speeds
        var driveVelocity = MathHelpers
                .pose2dFromRotation(currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(MathHelpers.transform2dFromTranslation(new Translation2d(driveVelocityScalar, 0.0)))
                .getTranslation();
        swerve.drive(driveVelocity, thetaVelocity, true);
        // swerve.setControl(new ApplyChassisSpeeds().withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
        //         driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation())));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveRobotRelative( new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return poseSupplier.get().equals(null) || (driveController.atGoal() && thetaController.atGoal());
    }
}
