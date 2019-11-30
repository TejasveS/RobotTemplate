package org.ghrobotics.frc2020.subsystems

import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import org.ghrobotics.frc2020.Constants
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.physics.MotorCharacterization
import org.ghrobotics.lib.subsystems.drive.FalconWestCoastDrivetrain
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits as nativeUnits1

object Drivetrain2 : FalconWestCoastDrivetrain() {
    override val controller: RamseteController
        get() = TODO("not implemented") //To change initializer of created properties use File | Settings | File Templates.
    override val gyro: Source<Rotation2d>
        get() = TODO("not implemented") //To change initializer of created properties use File | Settings | File Templates.
    override val kinematics: DifferentialDriveKinematics
        get() = TODO("not implemented") //To change initializer of created properties use File | Settings | File Templates.
    override val leftCharacterization: MotorCharacterization<Meter>
        get() = TODO("not implemented") //To change initializer of created properties use File | Settings | File Templates.
    override val odometry: DifferentialDriveOdometry
        get() = TODO("not implemented") //To change initializer of created properties use File | Settings | File Templates.
    override val rightCharacterization: MotorCharacterization<Meter>
        get() = TODO("not implemented") //To change initializer of created properties use File | Settings | File Templates.

    override fun activateEmergency() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun recoverFromEmergency() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    // 1440 ticks per wheel rotation
    // wheel radius is 3 inches
    val nativeUnitModel = NativeUnitLengthModel(1440.nativeUnits1, 3.inches)
    override val leftMotor = FalconSRX(Constants.Drivetrain.kLeftMasterId, nativeUnitModel)
    override val rightMotor = FalconSRX(Constants.Drivetrain.kRightMasterId, nativeUnitModel)
    private val leftslave1 = FalconSRX(Constants.Drivetrain.kLeftSlave1Id, nativeUnitModel)
    private val rightslave1 = FalconSRX(Constants.Drivetrain.kRightSlave1Id, nativeUnitModel)

    init {
        leftslave1.follow(leftMotor)
        rightslave1.follow(rightMotor)

        leftMotor.outputInverted = false
        leftslave1.outputInverted = false

        rightslave1.outputInverted = true
        rightslave1.outputInverted = true
    }
}

