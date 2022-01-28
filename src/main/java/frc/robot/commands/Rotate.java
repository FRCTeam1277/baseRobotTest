// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Rotate extends CommandBase {
  double distance;
  double distancePerTick;
  DriveTrain driveTrain;
  PIDController controller;
  double target;
  /** Creates a new Rotate. */
  public Rotate(DriveTrain dt, double distInches) {
    driveTrain =dt;
    addRequirements(dt);
    distancePerTick = Math.PI*8/4096;
    distance = distInches/distancePerTick;

    controller = new PIDController(0.01, 0,0);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetGyro();
    driveTrain.resetEncoders();
    controller.setSetpoint(0);
    // target = driveTrain.getGyro() + 90;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double out = -controller.calculate((driveTrain.getGyro()-target+180)%360 - 180);
    double l = 0.2;
    double out = controller.calculate(driveTrain.getGyro());
    //arcade drive thing fix later
    out = Math.min(Math.max(out, -l), l);
    driveTrain.updateMotors(0.4+out, 0.4+-out);
    System.out.println();
    System.out.println(out);
    System.out.println(driveTrain.getLeftEncoder());
    System.out.println(driveTrain.getRightEncoder());

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.updateMotors(0, 0);
    driveTrain.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveTrain.getLeftEncoder()-driveTrain.getRightEncoder())/2 > distance;
  }
}
