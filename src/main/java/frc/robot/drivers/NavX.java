// package org.frcteam2910.common.robot.drivers;
package frc.robot.drivers;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
// import org.frcteam2910.common.drivers.Gyroscope;
// import org.frcteam2910.common.math.Rotation2;

public final class NavX extends Gyroscope {
    private static AHRS navX;

    public NavX(SPI.Port port) {
        this(port, (byte) 200);
    }

    public NavX(SPI.Port port, byte updateRate) {
        navX = new AHRS(port, updateRate);
    }

    public NavX(SerialPort.Port port) {
        this(port, (byte) 200);
    }

    public NavX(SerialPort.Port port, byte updateRate) {
        navX = new AHRS(port, AHRS.SerialDataType.kProcessedData, updateRate);
    }

    @Override
    public void calibrate() {
        navX.reset();
    }

    @Override
    public Rotation2d getUnadjustedAngle() {
        // System.out.println("getUnadjustedAngle: " + getAxis(Axis.YAW));
        return Rotation2d.fromRadians(getAxis(Axis.YAW));
    }

    public Rotation2d getRotation2d() {
        // System.out.println("getRotation2d: " + getAxis(Axis.YAW));
        // had to invert the get axis to fir the relative feild velocity comands after a 90 degree rotation
        return Rotation2d.fromRadians(-getAxis(Axis.YAW));
    }

    public static double getRawAccelX() {
        return navX.getRawAccelX();
    }

    public static double getRawGyroY() {
        return navX.getRawGyroY();
    }

    @Override
    public double getUnadjustedRate() {
        return Math.toRadians(navX.getRate());
    }

    public double getAxis(Axis axis) {
        switch (axis) {
            case PITCH:
                return Math.toRadians(navX.getPitch());
            case ROLL:
                return Math.toRadians(navX.getRoll());
            case YAW:
                return Math.toRadians(navX.getYaw());
            default:
                return 0.0;
        }
    }

    public enum Axis {
        PITCH,
        ROLL,
        YAW
    }
}
