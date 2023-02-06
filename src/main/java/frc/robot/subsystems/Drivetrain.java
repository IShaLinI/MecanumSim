// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
      new Translation2d(-0.258571, -0.291841));

  MecanumDrivePoseEstimator mPoseEstimator = new MecanumDrivePoseEstimator(
      mKinematics,
      mPigeon.getRotation2d(),
      getCurrentDistances(),
      new Pose2d(new Translation2d(4, 4), new Rotation2d()));

  private final MecanumSimulation mSimulation = new MecanumSimulation(
    new TalonFXSimCollection[]{
      mFrontLeft.getSimCollection(),
      mFrontRight.getSimCollection(),
      mBackLeft.getSimCollection(),
      mBackRight.getSimCollection()
    },
    mPigeon.getSimCollection(),
    Constants.DRIVETRAIN_CHARACTERIZATION,
    mKinematics,
    DCMotor.getFalcon500(1),
    KitbotGearing.k10p71.value,
    this::getWheelSpeeds,
    this::getMotorSets
  );

  SimpleMotorFeedforward mFeedForward = new SimpleMotorFeedforward(Constants.DRIVETRAIN_kS,Constants.DRIVETRAIN_kV,Constants.DRIVETRAIN_kA);

  PIDController mFrontLeftPIDController = new PIDController(0.4, 0, 0);
  PIDController mFrontRightPIDController = new PIDController(0.4, 0, 0);
  PIDController mBackLeftPIDController = new PIDController(0.4, 0, 0);
  PIDController mBackRightPIDController = new PIDController(0.4, 0, 0);

  private Field2d mField = new Field2d();

  public Drivetrain() {

    SmartDashboard.putData("Field", mField);

  }

  public double[] getMotorSets() {
    return new double[] {
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

  public MecanumDriveWheelPositions getCurrentDistances() {
    return new MecanumDriveWheelPositions(
        mFrontLeft.getSelectedSensorPosition() * Constants.kFalconToMeters,
        mFrontRight.getSelectedSensorPosition() * Constants.kFalconToMeters,
        mBackLeft.getSelectedSensorPosition() * Constants.kFalconToMeters,
        mBackRight.getSelectedSensorPosition() * Constants.kFalconToMeters);
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {

    SmartDashboard.putNumber("Wheel Velocity Setpoints/FrontLeft", speeds.frontLeftMetersPerSecond);
    SmartDashboard.putNumber("Wheel Velocity Setpoints/FrontRight", speeds.frontRightMetersPerSecond);
    SmartDashboard.putNumber("Wheel Velocity Setpoints/BackLeft", speeds.rearLeftMetersPerSecond);
    SmartDashboard.putNumber("Wheel Velocity Setpoints/BackRight", speeds.rearRightMetersPerSecond);
    
    final double frontLeftFeedForward = mFeedForward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedForward = mFeedForward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedForward = mFeedForward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedForward = mFeedForward.calculate(speeds.rearRightMetersPerSecond);

    final double frontLeftOutput = mFrontLeftPIDController.calculate(
        mFrontLeft.getSelectedSensorVelocity() * Constants.kFalconToMeters * 10,
        speeds.frontLeftMetersPerSecond);

    final double frontRightOutput = mFrontRightPIDController.calculate(
        mFrontRight.getSelectedSensorVelocity() * Constants.kFalconToMeters * 10,
        speeds.frontRightMetersPerSecond);

    final double backRightOutput = mBackRightPIDController.calculate(
        mBackRight.getSelectedSensorVelocity() * Constants.kFalconToMeters * 10,
        speeds.rearRightMetersPerSecond);

    final double backLeftOutput = mBackLeftPIDController.calculate(
        mBackLeft.getSelectedSensorVelocity() * Constants.kFalconToMeters * 10,
        speeds.rearLeftMetersPerSecond);

    mFrontLeft.setVoltage(frontLeftFeedForward + frontLeftOutput);
    mFrontRight.setVoltage(frontRightFeedForward + frontRightOutput);
    mBackLeft.setVoltage(backLeftFeedForward + backLeftOutput);
    mBackRight.setVoltage(backRightFeedForward + backRightOutput);

  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var mecanumDriveWheelSpeeds =
        mKinematics.toWheelSpeeds(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, mPigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    setSpeeds(mecanumDriveWheelSpeeds);
  }

  @Override
  public void periodic() {

    mPoseEstimator.update(mPigeon.getRotation2d(), getCurrentDistances());

    SmartDashboard.putNumber("Wheel Velocities/FrontLeft", getWheelSpeeds().frontLeftMetersPerSecond);
    SmartDashboard.putNumber("Wheel Velocities/FrontRight", getWheelSpeeds().frontRightMetersPerSecond);
    SmartDashboard.putNumber("Wheel Velocities/BackLeft", getWheelSpeeds().rearLeftMetersPerSecond);
    SmartDashboard.putNumber("Wheel Velocities/BackRight", getWheelSpeeds().rearRightMetersPerSecond);

    mField.setRobotPose(mPoseEstimator.getEstimatedPosition());
  }

  @Override
  public void simulationPeriodic() {
    mSimulation.update();
  }
}
