// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private final TalonFX m_fx = new TalonFX(1, "rio"); 
    private final DutyCycleOut m_out = new DutyCycleOut(0);
     /* Start at position 0, enable FOC, no feed forward, use slot 0 */
    private final PositionVoltage m_voltagePosition = new PositionVoltage(0, 0, false, 0, 0, false, false, true);
    //Motion Magic
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    // create a Motion Magic Velocity request, voltage output
    private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    //Joystick
    private final XboxController m_joystick = new XboxController(0);
    boolean isHomed = false;
    boolean isPositive = false;
    boolean isMotionMagic = false;
    double dSetpoint = 0.0;
    
    /* Keep a brake request so we can disable the motor */
    private final NeutralOut m_brake = new NeutralOut();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    //Used for the homing of the mech
    configs.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    configs.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    configs.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
    //Used for the voltage closed loop motion
    Slot0Configs slot0 = configs.Slot0;
    //configs.GravityType = GravityType.
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kP = 0.63; // An error of 0.5 rotations results in 1.2 volts output
    slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    /** *********************************************************************************************
     *  Motion Magic
    /* Configure current limits */
    MotionMagicConfigs mm = configs.MotionMagic;
    mm.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    mm.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    Slot1Configs slot1 = configs.Slot1;
    slot1.GravityType = GravityTypeValue.Arm_Cosine;
    slot1.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot1.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot1.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot1.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot1.kI = 0; // no output for integrated error
    slot1.kD = 0; // no output for error derivative
    
   
    //FeedbackConfigs fdb = configs.Feedback;
    //fdb.SensorToMechanismRatio = 1;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    //testing only
    SmartDashboard.putBoolean("Run Motor", false);
    SmartDashboard.putBoolean("Direction", false);
    SmartDashboard.putNumber("Setpoint", 10);
    SmartDashboard.putBoolean("Run Motion Magic", false);
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
    //Updates position on the dashboard
    StatusSignal<Double> dPosition = m_fx.getPosition();
    StatusSignal<Double> dVelocity = m_fx.getVelocity();
    StatusSignal<Boolean> reverseLimitStatus = m_fx.getFault_ReverseHardLimit(); 
    SmartDashboard.putNumber("Position", dPosition.getValue());
    SmartDashboard.putNumber("Velocity", dVelocity.getValue());
    SmartDashboard.putBoolean("Reverse Limit", reverseLimitStatus.getValue());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   */
  @Override
  public void autonomousInit() {
      m_fx.setControl(m_out.withOutput(-0.10));
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Retrieve off the dashboard values
    isHomed = SmartDashboard.getBoolean("Run Motor", false);
    isPositive = SmartDashboard.getBoolean("Direction",false );
    dSetpoint = SmartDashboard.getNumber("Setpoint", kDefaultPeriod);
    isMotionMagic = SmartDashboard.getBoolean("Run Motion Magic", isMotionMagic);

    if (isHomed == true) {
      if (isPositive == true){
        m_fx.setControl(m_out.withOutput(0.1));
      }
      else {
        m_fx.setControl(m_out.withOutput(-0.2));
      }
    }
    //Disabled Motor
    if (isHomed == false) {
      m_fx.setControl(m_out.withOutput(0));
    }

    // Motion to postion
    if (m_joystick.getLeftBumper()) {
        /* Use voltage position */
        m_fx.setControl(m_voltagePosition.withPosition(dSetpoint));
     }
      else {
        /* Disable the motor instead */
       // m_fx.setControl(m_brake);
    }

    // Motion magic to postion by voltage
    if (m_joystick.getRightBumper()) {
       // Double dDistance = dSetpoint * 2048;
        /* Use voltage position */
        m_fx.setControl(m_mmReq.withPosition(dSetpoint).withSlot(1));
     }
      else {
        /* Disable the motor instead */
      //  m_fx.setControl(m_brake);
    }

    // Motion with motion magic to position by velocity
    if (m_joystick.getAButtonPressed()) {
      m_fx.setControl(m_request.withVelocity(kDefaultPeriod).withSlot(1));
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}           
}
