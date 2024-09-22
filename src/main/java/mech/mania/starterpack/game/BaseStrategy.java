package mech.mania.starterpack.game;

import java.util.Map;

// This defines the general layout your strategy method will inherit
// 
// DO NOT EDIT THIS!

public abstract class BaseStrategy {
    protected String team;

    public BaseStrategy(String team) {
        this.team = team;
    }

    /**
     * Return a map mapping PlaneType to int
     */
    public abstract Map<PlaneType, Integer> selectPlanes();

    /**
     * Return a map mapping each plane id to the amount they will steer [-1, 1],
     * where positive is clockwise
     */
    public abstract Map<String, Double> steerInput(Map<String, Plane> planes);
}