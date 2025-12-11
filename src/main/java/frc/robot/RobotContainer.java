// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.swerve.Module;
import frc.robot.subsystems.swerve.ModuleIOSpark;

import static frc.robot.constants.Constants.*;

public class RobotContainer {

  private DriveSubsystem drive;
  private XboxController xboxCon1 = new XboxController(0);

  public RobotContainer() {

    drive = new DriveSubsystem(
      new Module(new ModuleIOSpark(21, 11)),
      new Module(new ModuleIOSpark(22, 12)),
      new Module(new ModuleIOSpark(23, 13)),
      new Module(new ModuleIOSpark(24, 14))
    );
    
    configureBindings();
  }

  private void configureBindings() {
    new Trigger(() -> xboxCon1.getAButton()).onTrue(new InstantCommand(() -> {
      drive.zero();
    }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void periodic(){
    if(Math.hypot(xboxCon1.getLeftX(), xboxCon1.getLeftY()) > 0.15 || Math.abs(xboxCon1.getRightX()) > 0.15){
      drive.drive(new ChassisSpeeds(
        MathUtil.applyDeadband(Math.pow(xboxCon1.getLeftY() , 1), 0.15)*MAX_LINEAR_SPEED,
        MathUtil.applyDeadband(Math.pow(xboxCon1.getLeftX() , 1), 0.15)*MAX_LINEAR_SPEED,
        MathUtil.applyDeadband(Math.pow(xboxCon1.getRightX(), 1), 0.15)*MAX_ANGULAR_SPEED
      ));
    } else {
      drive.drive(new ChassisSpeeds());
    }
  }
}
