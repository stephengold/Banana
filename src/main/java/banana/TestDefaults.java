/*
 Copyright (c) 2020, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package banana;

import com.jme3.app.SimpleApplication;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.GImpactCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.SliderJoint;
import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.bullet.joints.motors.TranslationalLimitMotor;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.bullet.objects.VehicleWheel;
import com.jme3.math.FastMath;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.shape.Quad;
import com.jme3.scene.shape.Torus;
import java.util.logging.Logger;

/**
 * Verify the default parameters of various physics objects.
 */
public class TestDefaults extends SimpleApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(TestDefaults.class.getName());
    // *************************************************************************
    // fields

    private BoxCollisionShape box; // initialized by testShapes()
    private PhysicsRigidBody rigidA;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestDefaults application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        TestDefaults application = new TestDefaults();
        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        PhysicsSpace pSpace = new PhysicsSpace(
                new Vector3f(-10000f, -10000f, -10000f),
                new Vector3f(10000f, 10000f, 10000f),
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3);
        testPhysicsSpace(pSpace);

        testShapes();
        // TODO GhostControl, CharacterControl, RigidBodyControl, VehicleControl

        PhysicsCharacter character = new PhysicsCharacter(box, 1f);
        testPco(character);
        assertEquals(55f, character.getFallSpeed(), 0f);
        assertEquals(10f, character.getJumpSpeed(), 0f);
        assertEquals(FastMath.QUARTER_PI, character.getMaxSlope(), 0f);
        assertEquals(0f, 0f, 0f, character.getPhysicsLocation(null), 0);

        PhysicsGhostObject ghost = new PhysicsGhostObject(box);
        testPco(ghost);

        PhysicsRigidBody srb = new PhysicsRigidBody(box, 0f);
        testRigidBody(srb);

        rigidA = new PhysicsRigidBody(box);
        testRigidBody(rigidA);

        PhysicsVehicle vehicle = new PhysicsVehicle(box);
        testRigidBody(vehicle);
        assertEquals(10.5f, vehicle.getFrictionSlip(), 0f);
        assertEquals(6000f, vehicle.getMaxSuspensionForce(), 0f);
        assertEquals(500f, vehicle.getMaxSuspensionTravelCm(), 0f);
        assertEquals(0, vehicle.getNumWheels());
        assertEquals(0.83f, vehicle.getSuspensionCompression(), 0f);
        assertEquals(0.88f, vehicle.getSuspensionDamping(), 0f);
        assertEquals(5.88f, vehicle.getSuspensionStiffness(), 0f);

        pSpace.add(vehicle);
        assertEquals(0f, vehicle.getCurrentVehicleSpeedKmHour(), 0f);
        assertEquals(0f, 0f, 1f, vehicle.getForwardVector(null), 0f);

        Vector3f connectionPoint = Vector3f.ZERO;
        Vector3f direction = new Vector3f(0f, -1f, 0f);
        Vector3f axle = new Vector3f(-1f, 0f, 0f);
        float suspensionRestLength = 0.1f;
        float wheelRadius = 1f;
        boolean isFrontWheel = true;
        VehicleWheel wheel = vehicle.addWheel(null,
                connectionPoint, direction, axle, suspensionRestLength,
                wheelRadius, isFrontWheel);
        assertEquals(10.5f, wheel.getFrictionSlip(), 0f);
        assertEquals(500f, wheel.getMaxSuspensionTravelCm(), 0f);
        assertEquals(1f, wheel.getRollInfluence(), 0f);
        assertEquals(5.88f, wheel.getSuspensionStiffness(), 0f);
        assertEquals(0.83f, wheel.getWheelsDampingCompression(), 0f);
        assertEquals(0.88f, wheel.getWheelsDampingRelaxation(), 0f);
        assertFalse(wheel.isApplyLocal());

        testJoints();

        stop();
    }
    // *************************************************************************
    // private methods

    void assertEquals(boolean actual, boolean expected) {
        assert actual == expected;
    }

    void assertEquals(float actual, float expected, float tolerance) {
        assert Math.abs(actual - expected) <= tolerance;
    }

    void assertEquals(float x, float y, float z, float w, Quaternion quaternion,
            float tolerance) {
        assertEquals(x, quaternion.getX(), tolerance);
        assertEquals(y, quaternion.getY(), tolerance);
        assertEquals(z, quaternion.getZ(), tolerance);
        assertEquals(w, quaternion.getW(), tolerance);
    }

    void assertEquals(float x, float y, float z, Vector3f vector,
            float tolerance) {
        assertEquals(x, vector.x, tolerance);
        assertEquals(y, vector.y, tolerance);
        assertEquals(z, vector.z, tolerance);
    }

    void assertEquals(int expected, int actual) {
        assert actual == expected;
    }

    void assertFalse(boolean actual) {
        assert actual == false;
    }

    void assertNotEquals(long expected, long actual) {
        assert actual != expected;
    }

    void assertNotNull(Object actual) {
        assert actual != null;
    }

    void assertNull(Object actual) {
        assert actual == null;
    }

    void assertTrue(boolean actual) {
        assert actual == true;
    }

    private void testJoints() {
        PhysicsRigidBody rigidB = new PhysicsRigidBody(box);

        // TODO ConeJoint
        // TODO HingeJoint
        // TODO Point2PointJoint
        SixDofJoint deSix = new SixDofJoint(rigidA, rigidB, new Vector3f(),
                new Vector3f(), false);
        testSixDof(deSix, 2);

        SliderJoint deSlider = new SliderJoint(rigidA, rigidB,
                new Vector3f(0f, 0f, 0f), new Vector3f(0f, 0f, 0f), false);
        testSlider(deSlider, 2);
    }

    /**
     * Test the defaults that are common to all newly-created collision objects.
     *
     * @param pco the object to test (not null, unaffected)
     */
    private void testPco(PhysicsCollisionObject pco) {
        if (pco instanceof PhysicsRigidBody) {
            PhysicsRigidBody body = (PhysicsRigidBody) pco;
            assertEquals(0f, body.getCcdMotionThreshold(), 0f);
            assertEquals(0f, body.getCcdSweptSphereRadius(), 0f);
            assertEquals(0.5f, body.getFriction(), 0f);
            assertEquals(0f, 0f, 0f, body.getPhysicsLocation(), 0f);
            assertEquals(0f, body.getRestitution(), 0f);
        }
        assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                pco.getCollideWithGroups());
        assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                pco.getCollisionGroup());

        assertNull(pco.getUserObject());
    }

    /**
     * Verify defaults common to all newly-created physics spaces.
     *
     * @param space the space to test (not null, unaffected)
     */
    private void testPhysicsSpace(PhysicsSpace space) {
        assertEquals(1 / 60f, space.getAccuracy(), 0f);
        assertEquals(0f, -9.81f, 0f, space.getGravity(new Vector3f()), 0f);
        assertEquals(10, space.getSolverNumIterations());
    }

    /**
     * Test the defaults that are common to all newly-created rigid bodies.
     *
     * @param prb the body to test (not null, unaffected)
     */
    private void testRigidBody(PhysicsRigidBody prb) {
        assertNotNull(prb);
        testPco(prb);

        assertEquals(0f, prb.getAngularDamping(), 0f);
        assertEquals(1f, prb.getAngularFactor(), 0f);
        assertEquals(1f, prb.getAngularSleepingThreshold(), 0f);
        if (prb.getMass() > 0f) {
            assertEquals(0f, 0f, 0f, prb.getAngularVelocity(), 0);
            assertEquals(0f, 0f, 0f, prb.getLinearVelocity(), 0);
            assertEquals(1f, prb.getMass(), 0f);
        } else {
            assertEquals(0f, prb.getMass(), 0f);
        }
        assertFalse(prb.isKinematic());
    }

    /**
     * Verify the defaults for collision shapes.
     */
    private void testShapes() {
        /*
         * Box
         */
        box = new BoxCollisionShape(new Vector3f(1f, 1f, 1f));
        assertEquals(0.04f, box.getMargin(), 0f);
        /*
         * Capsule
         */
        CapsuleCollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        assertEquals(PhysicsSpace.AXIS_Y, capsule.getAxis());
        assertEquals(1f, capsule.getHeight(), 0f);
        assertEquals(0f, capsule.getMargin(), 0f);
        /*
         * Compound
         */
        CompoundCollisionShape compound = new CompoundCollisionShape();
        assertEquals(0.04f, compound.getMargin(), 0f);
        /*
         * Cone
         */
        ConeCollisionShape cone = new ConeCollisionShape(1f, 1f);
        assertEquals(1f, cone.getHeight(), 0f);
        assertEquals(0.04f, cone.getMargin(), 0f);
        /*
         * Cylinder
         */
        CylinderCollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(1f, 1f, 1f));
        assertEquals(PhysicsSpace.AXIS_Z, cylinder.getAxis());
        assertEquals(0.04f, cylinder.getMargin(), 0f);
        /*
         * GImpact
         */
        Torus torus = new Torus(16, 16, 0.2f, 0.8f);
        GImpactCollisionShape gimpact = new GImpactCollisionShape(torus);
        /*
         * Heightfield
         */
        float[] nineHeights = {1f, 0f, 1f, 0f, 0.5f, 0f, 1f, 0f, 1f};
        HeightfieldCollisionShape hcs
                = new HeightfieldCollisionShape(nineHeights);
        assertEquals(0.04f, hcs.getMargin(), 0f);
        /*
         * Hull
         */
        float prismVertices[] = new float[]{
            1f, 1f, 1f,
            1f, 1f, -1f,
            -1f, 1f, 0f,
            1f, -1f, 1f,
            1f, -1f, -1f,
            -1f, -1f, 0f
        };
        HullCollisionShape hull = new HullCollisionShape(prismVertices);
        assertEquals(0.04f, hull.getMargin(), 0f);
        /*
         * Mesh
         */
        Quad quad = new Quad(1f, 1f);
        MeshCollisionShape mesh = new MeshCollisionShape(quad);
        assertEquals(0.04f, mesh.getMargin(), 0f);
        /*
         * Plane
         */
        Plane plane = new Plane(new Vector3f(0f, 1f, 0f), 0f);
        PlaneCollisionShape pcs = new PlaneCollisionShape(plane);
        assertEquals(0.04f, pcs.getMargin(), 0f);
        assertEquals(0f, pcs.getPlane().getConstant(), 0f);
        assertEquals(0f, 1f, 0f, pcs.getPlane().getNormal(), 0f);
        /*
         * Simplex of 1 vertex
         */
        SimplexCollisionShape simplex1
                = new SimplexCollisionShape(new Vector3f(0f, 0f, 0f));
        testSimplexShape(simplex1);
        /*
         * Simplex of 2 vertices
         */
        SimplexCollisionShape simplex2 = new SimplexCollisionShape(
                new Vector3f(1f, 0f, 0f), new Vector3f(-1, 0f, 0f));
        testSimplexShape(simplex2);
        /*
         * Simplex of 3 vertices
         */
        SimplexCollisionShape simplex3 = new SimplexCollisionShape(
                new Vector3f(0f, 1f, 1f),
                new Vector3f(1f, 1f, 0f),
                new Vector3f(1f, 0f, 1f)
        );
        testSimplexShape(simplex3);
        /*
         * Simplex of 4 vertices
         */
        SimplexCollisionShape simplex4 = new SimplexCollisionShape(
                new Vector3f(0f, 1f, 1f),
                new Vector3f(0f, 1f, -1f),
                new Vector3f(1f, -1f, 0f),
                new Vector3f(-1f, -1f, 0f)
        );
        testSimplexShape(simplex4);
        /*
         * Sphere
         */
        SphereCollisionShape sphere = new SphereCollisionShape(1f);
        assertEquals(0f, sphere.getMargin(), 0f);
    }

    /**
     * Test the defaults that are common to all newly-created simplex shapes.
     *
     * @param simplex the shape to test (not null, unaffected)
     */
    private void testSimplexShape(SimplexCollisionShape simplex) {
        assertEquals(0.04f, simplex.getMargin(), 0f);
    }

    private void testSixDof(SixDofJoint six, int numEnds) {
        RotationalLimitMotor rlm
                = six.getRotationalLimitMotor(PhysicsSpace.AXIS_X);
        assertEquals(1f, rlm.getDamping(), 0f);
        assertFalse(rlm.isEnableMotor());
        assertEquals(0.2f, rlm.getERP(), 0f);
        assertEquals(0.5f, rlm.getLimitSoftness(), 0f);
        assertEquals(300f, rlm.getMaxLimitForce(), 0f);
        assertEquals(6f, rlm.getMaxMotorForce(), 0f);
        assertEquals(0f, rlm.getTargetVelocity(), 0f);

        TranslationalLimitMotor tlm = six.getTranslationalLimitMotor();
        assertEquals(1f, tlm.getDamping(), 0f);
        assertEquals(0.7f, tlm.getLimitSoftness(), 0f);
        assertEquals(0f, tlm.getRestitution(), 0.5f);
    }

    private void testSlider(SliderJoint slider, int numEnds) {
        assertEquals(0f, slider.getDampingDirAng(), 0f);
        assertEquals(0f, slider.getDampingDirLin(), 0f);
        assertEquals(1f, slider.getDampingLimAng(), 0f);
        assertEquals(1f, slider.getDampingLimLin(), 0f);
        assertEquals(1f, slider.getDampingOrthoAng(), 0f);
        assertEquals(1f, slider.getDampingOrthoLin(), 0f);
        assertEquals(0f, slider.getLowerAngLimit(), 0f);
        assertEquals(1f, slider.getLowerLinLimit(), 0f);
        assertEquals(0f, slider.getMaxAngMotorForce(), 0f);
        assertEquals(0f, slider.getMaxLinMotorForce(), 0f);
        assertEquals(0.7f, slider.getRestitutionDirAng(), 0f);
        assertEquals(0.7f, slider.getRestitutionDirLin(), 0f);
        assertEquals(0.7f, slider.getRestitutionLimAng(), 0f);
        assertEquals(0.7f, slider.getRestitutionLimLin(), 0f);
        assertEquals(0.7f, slider.getRestitutionOrthoAng(), 0f);
        assertEquals(0.7f, slider.getRestitutionOrthoLin(), 0f);
        assertEquals(1f, slider.getSoftnessDirAng(), 0f);
        assertEquals(1f, slider.getSoftnessDirLin(), 0f);
        assertEquals(1f, slider.getSoftnessLimAng(), 0f);
        assertEquals(1f, slider.getSoftnessLimLin(), 0f);
        assertEquals(1f, slider.getSoftnessOrthoAng(), 0f);
        assertEquals(1f, slider.getSoftnessOrthoLin(), 0f);
        assertEquals(0f, slider.getTargetAngMotorVelocity(), 0f);
        assertEquals(0f, slider.getTargetLinMotorVelocity(), 0f);
        assertEquals(0f, slider.getUpperAngLimit(), 0f);
        assertEquals(-1f, slider.getUpperLinLimit(), 0f);
        assertFalse(slider.isPoweredAngMotor());
        assertFalse(slider.isPoweredLinMotor());
    }
}
