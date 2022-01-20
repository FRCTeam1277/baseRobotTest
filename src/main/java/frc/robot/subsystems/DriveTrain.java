// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;



// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class DriveTrain extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private WPI_TalonSRX l2;
    private WPI_TalonSRX l3;
    private WPI_TalonSRX r2;
    private WPI_TalonSRX r3;
    private WPI_TalonSRX leftMaster;
    private WPI_TalonSRX rightMaster;
    private DifferentialDrive differentialDrive1;

    private AHRS navXAhrs;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
    *
    */
    public DriveTrain() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        l2 = new WPI_TalonSRX(0);

        l3 = new WPI_TalonSRX(5);

        r2 = new WPI_TalonSRX(3);

        r3 = new WPI_TalonSRX(1);

        leftMaster = new WPI_TalonSRX(4);

        rightMaster = new WPI_TalonSRX(2);
        
        l2.follow(leftMaster);
        l3.follow(leftMaster);
        r2.follow(rightMaster);
        r3.follow(rightMaster);
        
        differentialDrive1 = new DifferentialDrive(leftMaster, rightMaster);
        addChild("Differential Drive 1", differentialDrive1);
        differentialDrive1.setSafetyEnabled(true);
        differentialDrive1.setExpiration(0.1);
        differentialDrive1.setMaxOutput(1.0);


        navXAhrs = new AHRS();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void arcadeDrive(double xaxisSpeed, double zaxisRotate)
    {
        differentialDrive1.arcadeDrive(xaxisSpeed, zaxisRotate);
    }


	public void updateMotors(double left, double right) {
        leftMaster.set(left);
        rightMaster.set(-right); //right is reversed :/
    }

    public int getLeftEncoder(){
        return leftMaster.getSensorCollection().getQuadraturePosition();
    }

    public int getRightEncoder(){
        return rightMaster.getSensorCollection().getQuadraturePosition();
    }

    public void resetEncoders() {
        leftMaster.getSensorCollection().setQuadraturePosition(0, 0);
        rightMaster.getSensorCollection().setQuadraturePosition(0, 0);

    }

    public double getGyro() {
        return navXAhrs.getAngle();
    }

    public void resetGyro(){
        navXAhrs.reset();
    }
}
