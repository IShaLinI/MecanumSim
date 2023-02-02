// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class MecanumSimulation {

    private TalonFXSimCollection mFrontLeftSimCollection;
    private TalonFXSimCollection mFrontRightSimCollection;
    private TalonFXSimCollection mBackLeftSimCollection;
    private TalonFXSimCollection mBackRightSimCollection;

    private DoubleSupplier mFrontLeftMotorSet;
    private DoubleSupplier mFrontRightMotorSet;
    private DoubleSupplier mBackLeftMotorSet;
    private DoubleSupplier mBackRightMotorSet;

    private BasePigeonSimCollection mBasePigeonSimCollection;

    private DCMotor mMotor;
    private double mGearRatio;
    private double mWheelDiameterMeters;
    private double kDistancePerPulse;

    private FlywheelSim mFrontLeftWheelSimulation;
    private FlywheelSim mFrontRightWheelSimulation;
    private FlywheelSim mBackLeftWheelSimulation;
    private FlywheelSim mBackRightWheelSimulation;

    private double dtSeconds;

    public MecanumSimulation(
        TalonFXSimCollection[] _motorSims, 
        DoubleSupplier[] _motorSets,
        BasePigeonSimCollection _pigeonSim,
        DCMotor _motor, 
        double _gearRatio, 
        double _wheelDiameterMeters,
        double _characterizationCoefficients[],
        double _dtSeconds
) {

        mFrontLeftSimCollection = _motorSims[0];
        mFrontRightSimCollection = _motorSims[1];
        mBackLeftSimCollection = _motorSims[2];
        mBackRightSimCollection = _motorSims[3];

        mBasePigeonSimCollection = _pigeonSim;

        mMotor = _motor;
        mGearRatio = _gearRatio;
        mWheelDiameterMeters = _wheelDiameterMeters;
        kDistancePerPulse = (mWheelDiameterMeters * Math.PI) / (2048.0 * mGearRatio);

        mFrontLeftWheelSimulation = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(
                        _characterizationCoefficients[1],
                        _characterizationCoefficients[2]),
                mMotor,
                mGearRatio);

        mFrontRightWheelSimulation = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(
                        _characterizationCoefficients[1],
                        _characterizationCoefficients[2]),
                mMotor,
                mGearRatio);

        mBackLeftWheelSimulation = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(
                        _characterizationCoefficients[1],
                        _characterizationCoefficients[2]),
                mMotor,
                mGearRatio);

        mBackRightWheelSimulation = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(
                        _characterizationCoefficients[1],
                        _characterizationCoefficients[2]),
                mMotor,
                mGearRatio);

        mFrontLeftMotorSet = _motorSets[0];
        mFrontRightMotorSet = _motorSets[1];
        mBackLeftMotorSet = _motorSets[2];
        mBackRightMotorSet = _motorSets[3];

        dtSeconds = _dtSeconds;
    }

    public void update() {
        mFrontLeftWheelSimulation.setInput(mFrontLeftMotorSet.getAsDouble() * RobotController.getBatteryVoltage());
        mFrontRightWheelSimulation.setInput(mFrontRightMotorSet.getAsDouble() * RobotController.getBatteryVoltage());
        mBackLeftWheelSimulation.setInput(mBackLeftMotorSet.getAsDouble() * RobotController.getBatteryVoltage());
        mBackRightWheelSimulation.setInput(mBackRightMotorSet.getAsDouble() * RobotController.getBatteryVoltage());

        mFrontLeftWheelSimulation.update(dtSeconds);
        mFrontRightWheelSimulation.update(dtSeconds);
        mBackLeftWheelSimulation.update(dtSeconds);
        mBackRightWheelSimulation.update(dtSeconds);

        mFrontLeftSimCollection.setIntegratedSensorRawPosition(mFrontLeftWheelSimulation.)


    }

}
