package mech.mania.starterpack.game;

import java.lang.Math;

public record Vector(double x, double y) {
    public Vector add(Vector other) {
        return new Vector(other.x + this.x, other.y + this.y);
    }
    public Vector sub(Vector other) {
        return new Vector(this.x - other.x, this.y - other.y);
    }
    public Vector mul(double s) {
        return new Vector(this.x * s, this.y * s);
    }
    public Vector neg() {
        return new Vector(-this.x, -this.y);
    }
    public double dot(Vector other) {
        return this.x*other.x + this.y*other.y;
    }
    public double norm() {
        return Math.sqrt(this.x*this.x + this.y*this.y);
    }
    public double distance(Vector other) {
        return Math.sqrt((this.x-other.x)*(this.x-other.x) + (this.y-other.y)*(this.y-other.y));
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
}