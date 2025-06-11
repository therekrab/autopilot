package com.vendor.jni;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

public class VendorJNITest {
  @Test
  void testNumber() {
    // Test to verify that the JNI test link works correctly.
    assertEquals(VendorJNI.getNumber(), -1, "sin(pi) = -1");
  }
}
