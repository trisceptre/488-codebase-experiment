package first.robot.util.motor;

public record CANId(int id, String bus) {
    public CANId(int id) {
        this(id, "");
    }
}
