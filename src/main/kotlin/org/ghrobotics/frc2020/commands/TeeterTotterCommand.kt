/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import edu.wpi.first.wpilibj.PIDController
import edu.wpi.first.wpilibj.PIDOutput
import edu.wpi.first.wpilibj.PIDSource
import edu.wpi.first.wpilibj.PIDSourceType
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.ghrobotics.frc2020.subsystems.Drivetrain
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.volts
import kotlin.math.abs

/**
 * Command to drive the robot using the Xbox controller in teleop.
 */

object navxpitch : PIDSource{
    override fun getPIDSourceType(): PIDSourceType {
        return PIDSourceType.kDisplacement
    }

    override fun setPIDSourceType(pidSource: PIDSourceType?) {
    }

    override fun pidGet(): Double {
        SmartDashboard.putNumber("PID_Roll", Drivetrain.navx.roll.toDouble())
        return Drivetrain.navx.roll.toDouble()
    }
}

object ttdrive : PIDOutput{
    override fun pidWrite(output: Double) {
        SmartDashboard.putNumber("PIDOutput", output)
        println("PID Output:$output")
        Drivetrain.setPercent(output,output)
    }



}

class TeeterTotterCommand : FalconCommand(Drivetrain) {
    // logic 1
    var startClimb = false

    // logic 2
    var p = 0.5
    var i = 0.3
    var d = 0.5//0.10
    var f = 1.0
    var period = .005
    var pidCtrl: PIDController
    var started = false

    // common
    val setPoint = 0.0


    init {
        // logic 2
        pidCtrl = PIDController(p,i,d, navxpitch, ttdrive, period)
    }

    override fun initialize() {
        super.initialize()

        // logic 2
        pidCtrl.setOutputRange(-0.35,0.35)
        pidCtrl.setpoint = setPoint
        pidCtrl.setAbsoluteTolerance(0.05)
        pidCtrl.enable()
    }

    override fun execute() {
//        logic1()
        logic2()
    }

    fun logic1(){
        val kP = .65
        val kP1 = .45
        val kP2 = .35
        val threshold = 3.0
        val threshold1 = 5.0
        val threshold2 = 7.0

        println("roll:"+Drivetrain.navx.roll)
        val error = Drivetrain.navx.roll - setPoint
        if(Drivetrain.navx.roll > threshold){
            println("startClimb True")
            startClimb = true
        }
        startClimb = error >= 0

        if(startClimb) {
            var voltage = kP * error
//            if (Drivetrain.navx.roll > 1 && Drivetrain.navx.roll < -1) {
//                println("end/stop")
//                voltage = 0.0
//            }
            if(abs(Drivetrain.navx.roll) > threshold2 ) {
                println("peak")
                voltage = kP2 * error
            } else if (abs(Drivetrain.navx.roll) > threshold1){
                println("midway")
                voltage = kP1 * error
            } else {
                println("lowclimb")
                voltage = kP * error
            }
            println("voltage:$voltage")
//            Drivetrain.setPercent(voltage/12,voltage/12)
        } else {
            println("valley")
            // Drivetrain.setPercent(0.25,0.25)
        }
    }

    fun logic2(){
//        if(started && pidCtrl.onTarget()) {
//            pidCtrl.disable()
//            println("Stop")
//        } else if(!started){
//            pidCtrl.enable()
//            started=true
//            println("Started")
//        }
    }

    override fun end(interrupted: Boolean) {

        // logic 2
        if(pidCtrl.isEnabled) {
            pidCtrl.disable()
        }
    }
}
//abstract class FalconWestCoastDrive Train
//abstract val leftMotor falconmotor <Meter>
// abstract val leftMotor falconmotor <Meter>
// periodicIO
// overrde fun setneutral
//periodicIO.desireOutput = output nothing
//fun set output
//linear velocity linear acceleration
//
//val pitch error = drivetrain.get pitch
// arcadedrive
//linearpercent
//set percent
// periodicIO.desiredoutput = output.percent(left, right)
//periodIO.leftfeed forward = .volts
//periodicIO.rightfeedforward = .volts
//
//
//
//

/*
/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import org.ghrobotics.frc2020.subsystems.Drivetrain
import org.ghrobotics.lib.commands.FalconCommand
import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2020.Controls
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import kotlin.math.abs

/**
 * Command to drive the robot using the Xbox controller in teleop.
 */
class TeeterTotterCommand : FalconCommand(Drivetrain) {
    var pid_p = 0
    var pid_i = 0
    var pid_d = 0
    val elapsedTime = (time - timePrev) / 1000;

    var kp = 25
    var ki = 0
    var kd = 0.8
    var desired_angle = 0.0

    var Gyr_rawX= Controls.driverController.getX(GenericHID.Hand.kLeft)
    var Gyr_rawY= Controls.driverController.getY(GenericHID.Hand.kLeft)

    val Gyro_angle[0] = Gyr_rawX/131.0;
    val Gyro_angle[1] = Gyr_rawY/131.0;

    var Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
    var Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];

    var error = Total_angle[0] - desired_angle;
    val pid_p = kp*error;
    val pid_i = pid_i+(ki*error);
    val Pl pid_d = kd*((error - previous_error)/elapsedTime);
    vaID = pid_p + pid_d;
    var previous_error = error;
    var mspeed = abs(PID);
//    val kP = 0.15
//
//    val ttdrive = PIDOutput{
//        println(it)
//        Drivetrain.setPercent(-it,-it)
//    }

//    val navxpitch = PIDSource {
//        .navx.pitch as PIDSource
//    }

//        var pc : PIDController
//        init {
//            pc = PIDController(kP,0.0,0.0, Drivetrain.navx, ttdrive)
//
//            //        pc.setContinuous(true);
//     pc.setOutputRange(0.0,12.0)
//            pc.setpoint = 0.0
//        }

//        override fun initialize() {
//            super.initialize()
//            pc.enable()
//            println("initialized")
//        }
//
//        override fun end(interrupted: Boolean) {
//            pc.disable()
//        }

//    override fun execute() {
//        val pctval : Double = pc.get()/12
//        Drivetrain.setPercent(pctval, pctval )
//    }
    }

 */

