package frc.robot.subsystems;

import frc.robot.Constants.*;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Oiiai extends SubsystemBase{
    // private final Orchestra orchestra = new Orchestra();
    // private final TalonFX motor1 = new TalonFX(SwerveConstants.MOD0_CONSTANTS.DriveMotorId, RobotConstants.CANBUS_NAME);
    // private final TalonFX motor2 = new TalonFX(SwerveConstants.MOD1_CONSTANTS.DriveMotorId);
    // private final TalonFX motor3 = new TalonFX(SwerveConstants.MOD2_CONSTANTS.DriveMotorId);
    // private final TalonFX motor4 = new TalonFX(SwerveConstants.MOD3_CONSTANTS.DriveMotorId);

    Timer timer = new Timer();

    public Oiiai () {
        // orchestra.addInstrument(motor1);
        // orchestra.addInstrument(motor2);
        // orchestra.addInstrument(motor3);
        // orchestra.addInstrument(motor4);

        // var status = orchestra.loadMusic("output.chrp");
        // if (!status.isOK()) throw new RuntimeException("fuck it");


        timer.restart();
    }

    @Override
    public void periodic() {
    }

}
