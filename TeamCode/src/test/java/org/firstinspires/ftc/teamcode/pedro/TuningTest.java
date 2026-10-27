package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.junit.jupiter.api.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * The tuners in {@link Tuning} pass the checks Pedro's TunerScanner makes at startup (a bad one
 * throws there and no tuner is listed): static, no parameters, returns Procedure, and each builds
 * its procedure without touching hardware.
 */
class TuningTest {

    @Test
    void everyTunerIsOneTheScannerAcceptsAndBuilds() throws Exception {
        List<String> names = new ArrayList<>();
        for (Method m : Tuning.class.getDeclaredMethods()) {
            Tuner t = m.getAnnotation(Tuner.class);
            if (t == null) continue;
            assertTrue(Modifier.isStatic(m.getModifiers()), m.getName() + " is not static");
            assertEquals(0, m.getParameterCount(), m.getName() + " takes parameters");
            assertEquals(Procedure.class, m.getReturnType(), m.getName() + " does not return Procedure");
            assertNotNull(m.invoke(null), m.getName() + " built no procedure");
            names.add(t.name());
        }
        names.sort(null);
        assertEquals(Arrays.asList("1. Mecanum Tuner", "2. Pinpoint Tuner", "3. Foresight Tuner", "4. Tests"), names);
    }
}
