package mech.mania.starterpack.network;

public enum Phase {
    HELLO_WORLD("HELLO_WORLD"),
    PLANE_SELECT("PLANE_SELECT"),
    STEER_INPUT("STEER_INPUT"),
    FINISH("FINISH");
    private final String value;

    Phase(String value) {
        this.value = value;
    }

    public String getValue() {
        return value;
    }
}
