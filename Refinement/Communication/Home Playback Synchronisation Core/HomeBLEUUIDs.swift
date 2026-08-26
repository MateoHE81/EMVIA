import Foundation

/// UUID contract shared with `HomeSide_Feedback_Final.ino`.
/// Keep these values unchanged unless both the iOS and ESP32 implementations
/// are updated together.
public enum HomeBLEUUIDs {
    public static let deviceNamePrefix = "HomeFeedback-"

    public static let service =
        "8e3a0001-7f30-4e7f-a9e4-8f5e8a3d1c01"

    public static let feedbackCommand =
        "8e3a0002-7f30-4e7f-a9e4-8f5e8a3d1c01"

    public static let commandAcknowledgement =
        "8e3a0003-7f30-4e7f-a9e4-8f5e8a3d1c01"

    public static let homeTelemetry =
        "8e3a0004-7f30-4e7f-a9e4-8f5e8a3d1c01"

    public static let maintenanceControl =
        "8e3a0005-7f30-4e7f-a9e4-8f5e8a3d1c01"
}
