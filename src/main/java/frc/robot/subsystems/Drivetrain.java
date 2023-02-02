// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  
  public final WPI_TalonFX mFrontLeft = new WPI_TalonFX(Constants.FRONT_LEFT_ID);
  public final WPI_TalonFX mFrontRight = new WPI_TalonFX(Constants.FRONT_RIGHT_ID);
  public final WPI_TalonFX mBackLeft = new WPI_TalonFX(Constants.BACK_LEFT_ID);
  public final WPI_TalonFX mBackRight = new WPI_TalonFX(Constants.BACK_RIGHT_ID);

  public final WPI_Pigeon2 mPigeon = new WPI_Pigeon2(Constants.PIGEON_ID);

  public final MecanumDriveKinematics mKinematics = new MecanumDriveKinematics(
    new Translation2d(0.258571, 0.291841), 
    new Translation2d(0.258571, -0.291841), 
    new Translation2d(-0.258571, 0.291841), 
    new Translation2d(-0.258571, -0.291841)
  );

  public final MecanumDriveOdometry mOdometry = new MecanumDriveOdometry(mKinematics, mPigeon.getRotation2d(), new MecanumDriveWheelPositions());

  public Drivetrain() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
