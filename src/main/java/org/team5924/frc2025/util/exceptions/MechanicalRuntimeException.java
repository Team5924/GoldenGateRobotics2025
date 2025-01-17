package org.team5924.frc2025.util.exceptions;

public class MechanicalRuntimeException extends RuntimeException {
    private static final String DEFAULT_MSG = "default msg";

    public MechanicalRuntimeException() {
        super(DEFAULT_MSG);
    }

    public MechanicalRuntimeException(String errorMessage) {
        super(errorMessage);
    }

    public MechanicalRuntimeException(String errorMessageFmtString, Object... robotStateFields) {
        super(String.format(errorMessageFmtString, robotStateFields) + "\n");
    }
}
