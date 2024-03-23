package frc.robot.lib.math;


import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

public class NRUnitsTest {

    @Test
    public void testConstrainDeg() {
        //test 0 degrees
        assertEquals(NRUnits.constrainDeg(0.0), 0.0, 0.0001);

        assertEquals(NRUnits.constrainDeg(180.0), 180.0, 0.0001);
        assertEquals(NRUnits.constrainDeg(360.0), 0.0, 0.0001);
        assertEquals(NRUnits.constrainDeg(540.0), 180.0, 0.0001);
        assertEquals(NRUnits.constrainDeg(720.0), 0.0, 0.0001);

        assertEquals(NRUnits.constrainDeg(-180.0), 180.0, 0.0001);
        assertEquals(NRUnits.constrainDeg(-360.0), 0.0, 0.0001);
        assertEquals(NRUnits.constrainDeg(-540.0), 180.0, 0.0001);
        assertEquals(NRUnits.constrainDeg(-720.0), 0.0, 0.0001);

        assertEquals(NRUnits.constrainDeg(90.0), 90.0, 0.0001);
        assertEquals(NRUnits.constrainDeg(-90.0), -90.0, 0.0001);
        assertEquals(NRUnits.constrainDeg(270.0), -90.0, 0.0001);
        assertEquals(NRUnits.constrainDeg(-270.0), 90.0, 0.0001);
    }

    @Test
    public void testConstrainRad() {
        //test 0 radians
        assertEquals(NRUnits.constrainRad(Math.PI * 0.0), 0.0, 0.0001);

        assertEquals(NRUnits.constrainRad(Math.PI * 0.5), Math.PI/2.0, 0.0001);
        assertEquals(NRUnits.constrainRad(Math.PI * 1.0), Math.PI, 0.0001);
        assertEquals(NRUnits.constrainRad(Math.PI * 1.5), Math.PI/-2.0, 0.0001);
        assertEquals(NRUnits.constrainRad(Math.PI * 2.0), 0.0, 0.0001);

        assertEquals(NRUnits.constrainRad(Math.PI * -0.5), Math.PI/-2.0, 0.0001);
        assertEquals(NRUnits.constrainRad(Math.PI * -1.0), Math.PI, 0.0001);
        assertEquals(NRUnits.constrainRad(Math.PI * -1.5), Math.PI/2.0, 0.0001);
        assertEquals(NRUnits.constrainRad(Math.PI * -2.0), 0.0, 0.0001);

        assertEquals(NRUnits.constrainRad(Math.PI * 0.25), Math.PI*0.25,0.0001);
        assertEquals(NRUnits.constrainRad(Math.PI * -0.25), Math.PI*-0.25,0.0001);
        assertEquals(NRUnits.constrainRad(Math.PI * 0.75), Math.PI*0.75,0.0001);
        assertEquals(NRUnits.constrainRad(Math.PI * -0.75), Math.PI*-0.75,0.0001);
    }
    @Test
    public void testLogConstrainRad() {
        assertEquals(NRUnits.logConstrainRad(0.0),0.0, 0.0001);

        assertEquals(NRUnits.logConstrainRad(Math.PI),Math.PI, 0.0001);
        assertEquals(NRUnits.logConstrainRad(-Math.PI),Math.PI, 0.0001);
    }
}
