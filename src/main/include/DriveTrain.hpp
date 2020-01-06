#ifndef DRIVE_TRAIN_HPP
#define DRIVE_TRAIN_HPP

#include <iostream>
#include <frc/Drive/DifferentialDrive.h>
#include "controller.hpp"
#include "rev/CANSparkMax.h"
#include "AHRS.h"
#include <frc/Drive/DifferentialDrive.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/DoubleSolenoid.h>
#include "AHRS.h"

/**
 * @class	    DriveTrain
 * @description Makes the main.cpp(Robot.cpp) more readable 
 * @notes	    This class should be updated for other versions as needed.
 */
  class DriveTrain{
public:
/**
	 * @constructor DriveTrain
	 * @description Construct the DriveTrain class.
	 * @param 	  TopleftMotor     -- CanSparkMax Speed controller (TopleftMotor) 
	 * @param	    TopRightMotor    -- CanSparkMax Speed controller (TopRightMotor)
	 * @param	    BottomLeftMotor  -- CanSparkMax Speed controller (BottomLeftMotor)
	 * @param 	  BottomRightMotor -- CanSparkMax Speed controller (BottomRightMotor)
   * @param     Driver           -- driver contoller      
   * @param     ahrs             -- the navx borad (Connect through the SPI::Port::kMXP})
   * @param     Left_Solenoid    -- the left side double solenoid
   * @param     Right_Solenoid   -- the right side double solenoid 
*/
DriveTrain(rev::CANSparkMax &TopLeftMotor ,
          rev::CANSparkMax &TopRightMotor, 
          rev::CANSparkMax &BottomLeftMotor, 
          rev::CANSparkMax &BottomRightMotor, 
          FRC5572Controller &Driver,
          AHRS &ahrs,
          frc::DoubleSolenoid &Left_Solenoid,
          frc::DoubleSolenoid &Right_Solenoid);

~DriveTrain();
/*
	 * @description Drives the robot in crab mode
*/
void Drive();

/*
	 * @description Lowers the amps to 40A to reduce heat generation
*/
void LowerAmps();
/*
	 * @description shoots the pistons
*/
void POut();
/*
	 * @description retracts the pistons
*/
void PRetract();

private:
  frc::MecanumDrive* MRobotDrive;
  frc::SpeedControllerGroup* LeftMotors;
  frc::SpeedControllerGroup* RightMotors;
  FRC5572Controller* Driver;
  frc::DoubleSolenoid* Left_Solenoid;
  frc::DoubleSolenoid* Right_Solenoid;
  rev::CANSparkMax* TopleftMotor;
  rev::CANSparkMax* TopRightMotor; 
  rev::CANSparkMax* BottomLeftMotor; 
  rev::CANSparkMax* BottomRightMotor;
  AHRS* ahrs;
  };  
#endif