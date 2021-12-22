// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Rotate extends CommandBase {

  private DriveTrain driveTrain;
  private PIDController pidController;
  private double targetRot; //in degrees

  /** Creates a new Rotate. */
  public Rotate(DriveTrain driveTrain, double targetRot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.targetRot = targetRot;
    pidController = new PIDController(-.005, 0.0, 0);

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(targetRot);
    pidController.setTolerance(20);
  }

  @Override
  public void execute() {
    double leftSpeed = pidController.calculate(driveTrain.getLeftEncoder());
    double rightSpeed = pidController.calculate(driveTrain.getRightEncoder());

    driveTrain.updateMotors(leftSpeed,rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.updateMotors(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}

enum RotDirection {
  left,
  right
}