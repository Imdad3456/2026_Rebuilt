package frc.robot.util;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public class RobotUtilsTest {

    @Test
    void testCalculateLinearFactor() {
        // Test case 1: simple values
        double gearRatio1 = 10.0;
        double drumDiameter1 = 2.0;
        double expected1 = (Math.PI * drumDiameter1) / gearRatio1;
        assertEquals(expected1, RobotUtils.calculateLinearFactor(gearRatio1, drumDiameter1), 0.0001);

        // Test case 2: different values
        double gearRatio2 = 5.0;
        double drumDiameter2 = 4.0;
        double expected2 = (Math.PI * drumDiameter2) / gearRatio2;
        assertEquals(expected2, RobotUtils.calculateLinearFactor(gearRatio2, drumDiameter2), 0.0001);

        // Test case 3: zero drum diameter
        double gearRatio3 = 10.0;
        double drumDiameter3 = 0.0;
        double expected3 = 0.0; // (Math.PI * 0.0) / 10.0 = 0.0
        assertEquals(expected3, RobotUtils.calculateLinearFactor(gearRatio3, drumDiameter3), 0.0001);

        // Test case 4: gear ratio of 1
        double gearRatio4 = 1.0;
        double drumDiameter4 = 3.0;
        double expected4 = (Math.PI * drumDiameter4) / gearRatio4;
        assertEquals(expected4, RobotUtils.calculateLinearFactor(gearRatio4, drumDiameter4), 0.0001);
    }

    @Test
    void testToVelocityPerSecond() {
        // Test case 1: simple positive value
        double positionFactor1 = 60.0;
        double expected1 = 1.0; // 60.0 / 60.0 = 1.0
        assertEquals(expected1, RobotUtils.toVelocityPerSecond(positionFactor1), 0.0001);

        // Test case 2: zero value
        double positionFactor2 = 0.0;
        double expected2 = 0.0; // 0.0 / 60.0 = 0.0
        assertEquals(expected2, RobotUtils.toVelocityPerSecond(positionFactor2), 0.0001);

        // Test case 3: negative value
        double positionFactor3 = -120.0;
        double expected3 = -2.0; // -120.0 / 60.0 = -2.0
        assertEquals(expected3, RobotUtils.toVelocityPerSecond(positionFactor3), 0.0001);

        // Test case 4: fractional value
        double positionFactor4 = 30.0;
        double expected4 = 0.5; // 30.0 / 60.0 = 0.5
        assertEquals(expected4, RobotUtils.toVelocityPerSecond(positionFactor4), 0.0001);
    }

}
