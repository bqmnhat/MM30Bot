package mech.mania.starterpack.game;


import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;

@JsonIgnoreProperties(ignoreUnknown = true)
public class Plane {
    private final String id;
    private final String team;
    private final PlaneType type;
    private final Vector position;
    private final double angle;
    private final int health;
    private final PlaneStats stats;

    @JsonCreator
    public Plane(
            @JsonProperty("id") String id,
            @JsonProperty("team") String team,
            @JsonProperty("type") PlaneType type,
            @JsonProperty("position") Vector position,
            @JsonProperty("angle") double angle,
            @JsonProperty("health") int health,
            @JsonProperty("stats") PlaneStats stats) {
        this.id = id;
        this.team = team;
        this.type = type;
        this.position = position;
        this.angle = angle;
        this.health = health;
        this.stats = stats;
    }

    public String getId() {
        return id;
    }

    public String getTeam() {
        return team;
    }

    public PlaneType getType() {
        return type;
    }

    public Vector getPosition() {
        return position;
    }

    public double getAngle() {
        return angle;
    }

    public int getHealth() {
        return health;
    }

    public PlaneStats getStats() {
        return stats;
    }
}