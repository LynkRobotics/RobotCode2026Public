package frc.lib.util;

// Interface for any object that supports interpolation between two values
public interface Interpolatable<T> {
    // Weight is a value between 0.0 and 1.0 that refers to how heavily to weigh the current value
    T interpolate(double weight, T other);
}