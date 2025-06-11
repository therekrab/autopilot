package com.therekrab.autopilot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

public class LibraryTest {
  @Test
  void testNumber() {
    // Test to verify that the JNI test link works correctly.
    assertEquals(Library.getNumber(), -1, "sin(pi) = -1");
  }
}
