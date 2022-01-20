// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;

public class DriveForward extends CommandBase
{
  
  public DriveTrain driveTrain;
  private PIDController pid;
  private double distanceGoal;
  private double normalSpeed = 0.5;
  private double kMeterToEncoder = 0; //good luck with this

  public DriveForward(DriveTrain driveTrain, double distance) {
    
    this.driveTrain = driveTrain;
    this.distanceGoal = distance;
    pid = new PIDController(0.00025,0.0002,0);

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(10000);
    pid.setTolerance(100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double calc = pid.calculate(driveTrain.getLeftEncoder());
    calc = calc/2;
    //calc = (1.5/Math.PI) * Math.atan(calc/100);
    System.out.println(driveTrain.getLeftEncoder());
    driveTrain.updateMotors(calc, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.updateMotors(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
