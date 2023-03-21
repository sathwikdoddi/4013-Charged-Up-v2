// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(Constants.DeviceConstants.leftMotorID, MotorType.kBrushless);  
    private final CANSparkMax rightMotor = new CANSparkMax(Constants.DeviceConstants.rightMotorID, MotorType.kBrushless);  
    
    private final DutyCycleEncoder leftSideEncoder = new DutyCycleEncoder(Constants.DeviceConstants.leftEncoderPort); // needs to plug into DIO
    private final DutyCycleEncoder rightSideEncoder = new DutyCycleEncoder(Constants.DeviceConstants.rightEncoderPort); // needs to plug into DIO

    
    private final MotorControllerGroup leftSideMotors = new MotorControllerGroup(leftMotor);
    private final MotorControllerGroup rightSideMotors = new MotorControllerGroup(rightMotor);

    private static ADIS16448_IMU gyro_z;
    private final DifferentialDrive anumDrive = new DifferentialDrive(leftSideMotors, rightSideMotors);


    public Drivetrain() {
        gyro_z = new ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._256ms);

        leftSideMotors.setInverted(false);
        rightSideMotors.setInverted(false);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {}

    public void resetEncoders() {
        rightSideEncoder.reset();
        leftSideEncoder.reset();
    }

    public double getLeftPosition() {
        return leftSideEncoder.getDistance();
    }

    public double getRightPosition() {
        return rightSideEncoder.getDistance();
    }

    public void killAnumDrive() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public void controllerArcadeDrive(double forward, double turn, XboxController controller) {
        anumDrive.arcadeDrive(forward, turn);
        SmartDashboard.putNumber("Left Y // Forward", controller.getLeftY());
        SmartDashboard.putNumber("Right X // Turn", controller.getRightX());
        SmartDashboard.putNumber("Forward", forward);
        SmartDashboard.putNumber("Turn", turn);
    }

    public double getAxisAngle(String axis) {
        if (axis.equals("z")) {     return gyro_z.getAngle();    }
        return 0;
    }
}
