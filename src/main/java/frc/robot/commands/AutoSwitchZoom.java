/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.ShooterTiltSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;
import frc.robot.subsystems.HoodedShooterSubsystem;

public class AutoSwitchZoom extends CommandBase {
  /**
   * Creates a new AutoSwitchZoom.
   */

  private final LimeLight limelight;
  private final ShooterTurretSubsystem turret;
  private final ShooterTiltSubsystem tilt;
  private final HoodedShooterSubsystem shooter;

  private final double minUseableTargetHeight = 40;
  private final double maxUseableTargetHeight = 120;
  private final double maxUseableTargetWidth = 160;
  private boolean changeLocked;
  private int changeToZoom2Counter;
  private int changeToZoom1Counter;
  private int changeLockedCounter;
  private double switchTime = 1.2;
  private int counterLimit = (int) switchTime * 50;

  public AutoSwitchZoom(LimeLight limelight, ShooterTiltSubsystem tilt, ShooterTurretSubsystem turret,
      HoodedShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.tilt = tilt;
    this.turret = turret;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  /**
   * need to change between no and 2 x zoom as target gets too smallor large
   * otherwisw.
   * 
   * The tilt and turret lock loops should be stopped during the changeover.
   * 
   * 
   * 
   * 
   */
  public void execute() {

    SmartDashboard.putNumber("CZ1", changeToZoom1Counter);
    SmartDashboard.putNumber("CZ2", changeToZoom2Counter);
    SmartDashboard.putNumber("CL", changeLockedCounter);
    SmartDashboard.putBoolean("CLKD", changeLocked);

    // check if too far away for pipeline 0 - no zoom

    if (limelight.getIsTargetFound()) {

      if (limelight.getPipeline() == 0 && limelight.getBoundingBoxHeight() < minUseableTargetHeight && !changeLocked) {

        changeToZoom2Counter++;

      } else

      {
        changeToZoom2Counter = 0;
      }

      if (changeToZoom2Counter >= counterLimit) {

        limelight.setPipeline(1);

        changeLocked = true;

        changeLockedCounter = 10;

        changeToZoom2Counter = 0;
      }

      // check if too close for 2 x zoom
      if (limelight.getPipeline() == 1 && (limelight.getBoundingBoxHeight() > maxUseableTargetHeight
          || limelight.getBoundingBoxWidth() > maxUseableTargetWidth) && !changeLocked) {

        changeToZoom1Counter++;

      } else {

        changeToZoom1Counter = 0;
      }

      if (changeToZoom1Counter >= counterLimit) {

        limelight.setPipeline(0);

        changeLocked = true;

        changeLockedCounter = 10;

        changeToZoom1Counter = 0;
      }

      // if a chacgeover is in progress, wait for period of time
      if (changeLocked) {

        changeLockedCounter--;
      }

      if (changeLocked && changeLockedCounter <= 0) {

        changeLocked = false;
        changeLockedCounter = 0;

      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
