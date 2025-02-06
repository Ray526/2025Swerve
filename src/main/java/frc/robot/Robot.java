// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.revrobotics.jni.REVLibJNI;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.FSLib2025.util.LocalADStarAK;
import frc.FSLib2025.util.OSUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {
    private Command autonomousCommand = Commands.none();
    private Optional<Pose2d> startingPose = Optional.empty();
    private Command disabledCommand = Commands.none();
    private int mIter;
    private double timeOfLastSync = 0.0;
    private boolean hasBeenEnabledTeleop = false;
    private Optional<Alliance> allianceColor = Optional.of(Alliance.Blue);

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private Field2d m_field = new Field2d();

    public Robot() {
        // // always log to usb for now
        // Logger.addDataReceiver(new WPILOGWriter());
        // if (DriverStation.isFMSAttached()) {
        //     // Only turn on logs if we are attached to FMS, but disable NTtables.
        //     // Logger.addDataReceiver(new WPILOGWriter());
        // } else {
        //     // Otherwise, enable NT tables.
        //     Logger.addDataReceiver(new NT4Publisher());
        // }
        // Logger.start();

        // SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        m_robotContainer = new RobotContainer();
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathfindingCommand.warmupCommand().schedule();
        m_robotContainer.setRumble();
        // if (m_robotContainer.vision.getEstimatedGlobalPose().isPresent()) {
        // m_robotContainer.swerve.resetPosewithVision(m_robotContainer.vision.getEstimatedGlobalPose().get().estimatedPose.toPose2d());
        // }
    }

    @Override
    public void robotPeriodic() {
        // Switch thread to high priority to improve loop timing
        Threads.setCurrentThreadPriority(true, 99);
        // Return to normal thread priority
        // Threads.setCurrentThreadPriority(false, 10);

        // SmartDashboard.putBoolean("hasTag",
        // m_robotContainer.vision.getEstimatedGlobalPose().isPresent());
        CommandScheduler.getInstance().run();
        // m_robotContainer.addVision();

        m_field.setRobotPose(m_robotContainer.swerve.getOdometryPosition());
        SmartDashboard.putData("Swerve", m_field);
        SmartDashboard.putNumber("Volt", RobotController.getBatteryVoltage());
    }

    @Override
    public void disabledInit() {
        m_robotContainer.setRumble();
        m_robotContainer.swerve.stop();
        // disabledCommand = m_robotContainer.getDisabledCommand();
        // disabledCommand.schedule();
    }

    @Override
    public void disabledPeriodic() {
        // if (mIter % 50 == 0) {
        //     if (startingPose.isPresent() && m_robotContainer.odometryCloseToPose(startingPose.get())) {
        //         SmartDashboard.putBoolean("Near Auto Starting Pose", true);
        //     } else {
        //         SmartDashboard.putBoolean("Near Auto Starting Pose", false);
        //     }
        //     autonomousCommand = m_robotContainer.getAutonomousCommand();
        //     try {
        //         startingPose = PathPlannerAuto.getPathGroupFromAutoFile(autonomousCommand.getName()).get(0)
        //                 .getStartingHolonomicPose();
        //         m_robotContainer.m_field.getObject("startingPose").setPose(startingPose.get());
        //     } catch (IOException | ParseException e) {
        //         // TODO Auto-generated catch block
        //         e.printStackTrace();
        //     }
        //     if (startingPose.isPresent()) {
        //         m_robotContainer.swerve
        //                 .setOdometryPosition(startingPose.get());
        //     }
        //     allianceColor = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance() : allianceColor;

        //     // Logger.recordOutput("Regenerating Auto Command Timestamp", Timer.getFPGATimestamp());
        //     // Logger.recordOutput("Regenerating Auto Command", autonomousCommand.getName());

        //     // Someshit zero
        //     // if (robotContainer.getHood().getCurrentPositionRotations() < 0.0) {
        //     // robotContainer.getHood().resetZeroPoint();
        //     // }
        // }
        // mIter++;

        // if (hasBeenEnabledTeleop && (Timer.getFPGATimestamp() - timeOfLastSync) >= 10.0) {
        //     OSUtil.fsSyncAsync();
        //     timeOfLastSync = Timer.getFPGATimestamp();
        // }
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        // DataLogManager.start();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        // m_robotContainer.setRumble();
        // if (m_robotContainer.vision.hasTag()) {
        // m_robotContainer.swerve.resetPosewithVision(m_robotContainer.vision.getEstimatedGlobalPose().get().estimatedPose.toPose2d());
        // } SmartDashboard.putString("first pose",
        // m_robotContainer.vision.getEstimatedGlobalPose().get().estimatedPose.toPose2d().toString());
    }

    @Override
    public void teleopPeriodic() {
        // m_robotContainer.fixingPose();
    }

    @Override
    public void teleopExit() {
        m_robotContainer.setRumble();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
