package frc.team488.motor;

/**
 * A CAN device address.
 *
 * @param id device ID on the bus
 * @param bus CAN bus name; empty means the controller's default bus
 */
public record CanId(int id, String bus) {
    public CanId(int id) {
        this(id, "");
    }
}
