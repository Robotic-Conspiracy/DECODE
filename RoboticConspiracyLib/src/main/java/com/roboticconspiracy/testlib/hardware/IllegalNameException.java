package com.roboticconspiracy.testlib.hardware;

public class IllegalNameException extends RuntimeException {
    public IllegalNameException(String message) {
        super(message);
    }
    public IllegalNameException() {
        super("Name is either empty or null");
    }
}
