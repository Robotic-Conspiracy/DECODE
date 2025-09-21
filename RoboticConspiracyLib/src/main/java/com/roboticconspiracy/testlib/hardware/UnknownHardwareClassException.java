package com.roboticconspiracy.testlib.hardware;

public class UnknownHardwareClassException extends RuntimeException {
    public UnknownHardwareClassException(String message) {
        super(message);
    }

    public UnknownHardwareClassException(){
      super("Given hardware class is not a known or legal class");
    }
}
