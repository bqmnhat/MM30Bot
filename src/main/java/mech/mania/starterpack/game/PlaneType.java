package mech.mania.starterpack.game;

public enum PlaneType {
    STANDARD("STANDARD"),
    FLYING_FORTRESS("FLYING_FORTRESS"),
    THUNDERBIRD("THUNDERBIRD"),
    SCRAPYARD_RESCUE("SCRAPYARD_RESCUE"),
    PIGEON("PIGEON");
    private final String value;

    PlaneType(String value) {
            this.value = value;
        }

    public String getValue() {
        return value;
    }
}
