// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class MecanumSimulation {

    private TalonFXSimCollection mFrontLeftSimCollection;
    private TalonFXSimCollection mFrontRightSimCollection;
    private TalonFXSimCollection mBackLeftSimCollection;
    private TalonFXSimCollection mBackRightSimCollection;

    private BasePigeonSimCollection mBasePigeonSimCollection;

    private DCMotor mMotor;
    private double mGearRatio;

    private FlywheelSim mFrontLeftWheelSimulation;
    private FlywheelSim mFrontRightWheelSimulation;
    private FlywheelSim mBackLeftWheelSimulation;
    private FlywheelSim mBackRightWheelSimulation;

    public MecanumSimulation(TalonFXSimCollection[] _motorSims, BasePigeonSimCollection _pigeonSim, DCMotor _motor, double _gearRatio) {

        mFrontLeftSimCollection = _motorSims[0];
        mFrontRightSimCollection = _motorSims[1];
        mBackLeftSimCollection = _motorSims[2];
        mBackRightSimCollection = _motorSims[3];
        mBasePigeonSimCollection = _pigeonSim;
        mMotor = _motor;
        mGearRatio = _gearRatio;

        mFrontLeftWheelSimulation = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(
                Constants.DRIVETRAIN_kV,
                Constants.DRIVETRAIN_kA
            ),
            mMotor,
            mGearRatio
        );

        mFrontRightWheelSimulation = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(
                Constants.DRIVETRAIN_kV,
                Constants.DRIVETRAIN_kA
            ),
            mMotor,
            mGearRatio
        );

        mBackLeftWheelSimulation = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(
                Constants.DRIVETRAIN_kV,
                Constants.DRIVETRAIN_kA
            ),
            mMotor,
            mGearRatio
        );

        mBackRightWheelSimulation = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(
                Constants.DRIVETRAIN_kV,
                Constants.DRIVETRAIN_kA
            ),
            mMotor,
            mGearRatio
        );

    }

    public void update(){
        
    }

}
