// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.falcon;

public class poseDriveCommand extends Command {
  /** Creates a new poseDriveCommand. */
  private falcon motor;
  public poseDriveCommand(falcon f) {
    motor = f;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(f);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motor.runMotorPose(1000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
