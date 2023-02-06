// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MecanumSimulation;

public class Drivetrain extends SubsystemBase {
  
  private final WPI_TalonFX mFrontLeft = new WPI_TalonFX(Constants.FRONT_LEFT_ID);
  private final WPI_TalonFX mFrontRight = new WPI_TalonFX(Constants.FRONT_RIGHT_ID);
  private final WPI_TalonFX mBackLeft = new WPI_TalonFX(Constants.BACK_LEFT_ID);
  private final WPI_TalonFX mBackRight = new WPI_TalonFX(Constants.BACK_RIGHT_ID);

  private final WPI_Pigeon2 mPigeon = new WPI_Pigeon2(Constants.PIGEON_ID);

  private final MecanumDriveKinematics mKinematics = new MecanumDriveKinematics(
    new Translation2d(0.258571, 0.291841), 
    new Translation2d(0.258571, -0.291841), 
    new Translation2d(-0.258571, 0.291841), 
    new Translation2d(-0.258571, -0.291841)
  );

  private final MecanumDriveOdometry mOdometry = new MecanumDriveOdometry(mKinematics, mPigeon.getRotation2d(), new MecanumDriveWheelPositions());

  private final MecanumSimulation mSimulation = new MecanumSimulation(
    new TalonFXSimCollection[]{
      mFrontLeft.getSimCollection(),
      mFrontRight.getSimCollection(),
      mBackLeft.getSimCollection(),
      mBackRight.getSimCollection()
    },
    mPigeon.getSimCollection(),
    LinearSystemId.identifyVelocitySystem(Constants.DRIVETRAIN_kV, Constants.DRIVETRAIN_kA),
    mKinematics,
    DCMotor.getFalcon500(1),
    KitbotGearing.k10p71.value,
    this::getWheelSpeeds,
    this::getMotorSets
  );

  public Drivetrain() {
    
  }

  public double[] getMotorSets(){
    return new double[]{
      mFrontLeft.get(),
      mFrontRight.get(),
      mBackLeft.get(),
      mBackRight.get()
    };
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      mFrontLeft.getSelectedSensorVelocity() * Constants.kFalconToMeters * 10, 
      mFrontRight.getSelectedSensorVelocity() * Constants.kFalconToMeters * 10, 
      mBackLeft.getSelectedSensorVelocity() * Constants.kFalconToMeters * 10, 
      mBackRight.getSelectedSensorVelocity() * Constants.kFalconToMeters * 10);
  }


  public void set(double speed){
    if(Math.abs(speed) < 0.05){
      speed = 0;
    }

    mFrontLeft.set(speed);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    mSimulation.update();
  }
}
