// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  // private final DigitalOutput trig = new DigitalOutput(9);
  // private final AnalogInput echo = new AnalogInput(0);
  
  // distance in inches the robot wants to stay from an object
  // private static final double kHoldDistance = 12.0;

  // factor to convert sensor values to a distance in inches
  // private static final double kValueToInches = 0.125;

  // proportional speed constant
  // private static final double kP = 0.05;

  // private static final int kUltrasonicPort = 10;

  // median filter to discard outliers; filters over 10 sampleshttps://open.spotify.com/track/6p5CmR50Qd1e3Ouma2e8yx?si=36230758b6d94a47
  // private final MedianFilter m_filter = new MedianFilter(10);
  
  // private final DigitalInput m_ultrasonic = new DigitalInput(kUltrasonicPort);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // Shuffleboard.getTab("Example tab").add(trig);
    // Shuffleboard.getTab("Example tab").add(echo);    

    m_robotContainer = new RobotContainer();  
    m_robotContainer.trig.set(true);
    System.out.println("trig: " + m_robotContainer.trig.get());
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.   

    // double currentDistance = m_filter.calculate(m_ultrasonic.getValue()) * kValueToInches;
    // convert distance error to a motor speed
    //double currentSpeed = (kHoldDistance - currentDistance) * kP;

    // System.out.println(m_ultrasonic.get());
    for(var x = 0 ; x!=1000000000 ; x++){
      if(x == 100000000){
        //m_robotContainer.trig.pulse(100);
        System.out.println("echo: " + m_robotContainer.echo.getValue());
        // System.out.println("trig: " + trig.isPulsing());
        //trig.set(false);
        x=0;
      }
      // System.out.println("trig: " + trig.isPulsing());
    }

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Get selected routine from the SmartDashboard
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running which will
    // use the default command which is ArcadeDrive. If you want the autonomous
    // to continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
