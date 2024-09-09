package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.revrobotics.CANSparkBase;

import frc.robot.constants.SwerveConstants;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final SparkPIDController turnPID;

    private CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReverse;
    private final double encoderOffset;

    public SwerveModule(int driveMotorID, int turnMotorID, int absoluteEncoderID, boolean driveMotorReversed,
     boolean turnMotorReversed, boolean absoluteEncoderReversed, double encoderOffset) {

         this.encoderOffset = encoderOffset;
         this.absoluteEncoderReverse = absoluteEncoderReversed;

         absoluteEncoder = new CANcoder(absoluteEncoderID);

         CANcoderConfiguration config = new CANcoderConfiguration();

        driveMotor = new CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, CANSparkLowLevel.MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(SwerveConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.kDriveEncoderRPM2MeterPerSec);

        turnEncoder.setPositionConversionFactor(SwerveConstants.kTurningEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(SwerveConstants.kTurningEncoderRPM2RadPerSec);

        turnPID = turnMotor.getPIDController();
        turnPID.setP(SwerveConstants.kModuleTurningP);

        turnPID.setPositionPIDWrappingMinInput(-Math.PI);
        turnPID.setPositionPIDWrappingMaxInput(Math.PI);

        turnPID.setPositionPIDWrappingEnabled(true);

        turnPID.setFeedbackDevice(turnEncoder);

        driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        turnMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        resetEncoders();
     }

     public double getDrivePosition() {
         return driveEncoder.getPosition();
     }

     public double getTurnPosition() {
         return turnEncoder.getPosition();
     }

     public double getDriveVelocity() {
         return driveEncoder.getVelocity();
     }

     public double getTurnVelocity() {
         return turnEncoder.getVelocity();
     }

     public double getAbsolutePosition() {
         double angle = 360 * absoluteEncoder.getAbsolutePosition().getValue();
         angle -= encoderOffset;
         angle *= absoluteEncoderReverse ? -1 : 1;
         return angle;
     }
     public void resetEncoders() {
         driveEncoder.setPosition(0);
         turnEncoder.setPosition(0);
     }
     
}
