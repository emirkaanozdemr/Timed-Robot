// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(1);
  private WPI_VictorSPX leftSlave =  new WPI_VictorSPX(1);
  private WPI_VictorSPX rightSlave = new WPI_VictorSPX(2);

  private WPI_TalonSRX armMotor = new WPI_TalonSRX(5);
  private WPI_VictorSPX armSlave = new WPI_VictorSPX(3);

  private WPI_TalonSRX rollerMotor = new WPI_TalonSRX(4);

  private Compressor  compressor = new Compressor();
  private DoubleSolenoid hatchIntake = new DoubleSolenoid(0, 1); // PCM port 0, 1

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  // joysticks
  private Joystick driverJoystick = new Joystick(0);
  private Joystick operatorJoystick = new Joystick(1);

  // unit conversion
  private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;
  private final double kArmTick2Deg = 360. / 512 * 26 / 42 * 18 / 60 * 18 / 84;
    
  @Override
  public void robotInit() {
    // inverted settings
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    armMotor.setInverted(false);

    // slave setups
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    armSlave.follow(armMotor);

    leftSlave.setInverted(InvertType.FolloMaster);
    rightSlave.setInverted(InvertedType.FollowMaster);
    armSlave.setInverted(InvertType.FollowMaster);

    // init encoders
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(true);
    armMotor.setSensorPhase(true);

    // reset encoders to zero
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    armMotor.setSelectedSensorPosition(0, 0, 10);

    //set encoder boundary limits: to stop motors
    armMotor.configSelectedFeedbackSensor((int) (0 / kArmTick2Deg), 10);
    armMotor.configForwardSoftLimitThereshold((int) (175 / kArmTick2Deg), 10);

    armMotor.configForwardSoftLimitEnable(true, 10);
    armMotor.configForwardSoftLimitEnable(true, 10);

    // start compressor
    compressor.start();

    drive.setDeadband(0.05);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Arm Encoder Value",  armMotor.getSelectedSensorPosition() * kArmTick2Deg);
    SmartDashboard.putNumber("Left Dirve Encoder Value", leftMaster.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("Right Drive Encoder Value", rightMaster.getSelectedSensorPosition() * kDriveTick2Feet);  
  }

  @Override
  public void autonomousInit() {
    enableMotors(true);
    // reset encoders to zero
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    armMotor.setSelectedSensorPosition(0, 0, 10);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double leftPosititon = leftMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    double rightPosition = rightMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    double distance = (leftPosition + rightPosition) / 2;

    if (distance < 10){
      drive.tankDrive(0.6, 0.6);
      }
      else{
        drive.tankDrive(0, 0);
      }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    enableMotors(true);
  }
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  // driving
  double power = -driverJoystick.getRawAxis(1); // remember: negative sign
  double turn = driverJoystick.getRawAxis(4);

  /**
  // deadband
  if (Math.abs(power) < 0.05){
    power = 0;
  }
  if (Math.abs(turn) < 0.05){
    turn = 0;
  }
  **/
  drive.arcadeDrive(power * 0.6, turn * 0.3);

  // arm control
  double armPower = -operatorJoystick.getRawAxis(1); // remember negative sign
  if (Math.abs(armPower) < 0.05){
    armPower = 0;
  }
  armPower *= 0.5;
  armMotor.set(ControlMode.PercentOutput, armPower);

  // roller control
  double rollerPower = 0;
  if (operatorJoystick.getRawButton(0) == true){
    rollerPower  = -1;
  } else if (operatorJoystick.getRawButton(2)){
    rollerPower = -1;
  }
  rollerMotor.set(ControlMode.PercentOutput, rollerPower);

  // hatch intake
  if (operatorJoystick.getRawButton(3)){
    hatchIntake.set(Value.kReverse);
  } else{
    hatchIntake.set(Value.kForward);
  }

}

  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    enableMotors(false);
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void enableMotors(boolean on){
    NeutraLMode mode;
    if (on) {
      model = NeutralMode.Brake;
    }
    else {
      mode = NeutralMode.Coast;
    }
  leftMaster.setNeutralMode(mode);
  rightMaster.setNeutralMode(mode);
  leftSlave.setNeutralMode(mode);
  rightSlave.setNeutralMode(mode);
  armMotor.setNeutralMode(mode);
  armSlave.setNeutralMode(mode);
  rollerMotor.setNeutralMode(mode);
  }
}

