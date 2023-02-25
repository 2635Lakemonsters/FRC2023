package frc.robot.legacymath2910;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
