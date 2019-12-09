/*
 Copyright (c) 2019, Stephen Gold
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

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.PhysicsCollisionEvent;
import com.jme3.bullet.collision.PhysicsCollisionListener;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import java.util.logging.Logger;

/**
 * Test collision detection, including overlap of ghost objects.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestDetectCollision
        extends SimpleApplication
        implements PhysicsCollisionListener, PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * margin for collision shapes (affects narrow-phase collision checks only,
     * not applicable to all shapes)
     */
    final private float margin = 0.001f;
    /**
     * size (half extent) of the axis-aligned bounding boxes of all collision
     * shapes
     */
    final private float size = 1f;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestDetectCollision.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * when true, terminate this application
     */
    private boolean allTestsComplete = false;
    /**
     * sequence of collision shapes to test
     */
    private CollisionShape[] shapes;
    /**
     * distance at which overlap was 1st detected, or -1 if not yet detected
     */
    private float overlapDistance = -1f;
    /*
     * configuration for the next test (5 indices)
     */
    private int nextDirectionIndex = 0;
    private int nextFixedShapeIndex = 0;
    private int nextFixedTypeIndex = 0;
    private int nextMovingShapeIndex = 0;
    private int nextMovingTypeIndex = 0;
    /**
     * fixed collision object; null means "ready to start a new test"
     */
    private PhysicsCollisionObject fixedPco = null;
    /**
     * moving collision object; null means "ready to start a new test"
     */
    private PhysicsCollisionObject movingPco = null;
    /**
     * space used for physics simulation
     */
    private PhysicsSpace physicsSpace;
    /**
     * name of the direction-of-motion in the current test
     */
    private String describeDirectionOfMotion;
    /**
     * sequence of motion directions to test
     */
    private Vector3f[] directions;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        Application application = new TestDetectCollision();
        application.start();
    }
    // *************************************************************************
    // SimpleApplication methods

    /**
     * Initialize this application.
     */
    @Override
    public void simpleInitApp() {
        configureCamera();
        configurePhysics();

        CollisionShape box
                = new BoxCollisionShape(new Vector3f(size, size, size));
        box.setMargin(margin);

        CollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(size, size, size));
        cylinder.setMargin(margin);

        CollisionShape sphere = new SphereCollisionShape(size);

        Vector3f p1 = new Vector3f(+size, +size, +size);
        Vector3f p2 = new Vector3f(-size, -size, +size);
        Vector3f p3 = new Vector3f(-size, +size, -size);
        Vector3f p4 = new Vector3f(+size, -size, -size);
        CollisionShape tetrahedron = new SimplexCollisionShape(p1, p2, p3, p4);
        tetrahedron.setMargin(margin);

        float[] octVertices = new float[]{
            +size, 0f, 0f,
            -size, 0f, 0f,
            0f, +size, 0f,
            0f, -size, 0f,
            0f, 0f, +size,
            0f, 0f, -size
        };
        HullCollisionShape octahedron = new HullCollisionShape(octVertices);
        octahedron.setMargin(margin);

        shapes = new CollisionShape[]{
            sphere,
            box,
            cylinder,
            tetrahedron,
            octahedron
        };

        directions = new Vector3f[]{
            new Vector3f(1f, 1f, 0f).normalizeLocal(),
            new Vector3f(-1f, 0f, 0f),
            new Vector3f(1f, -3f, 0f).normalizeLocal()
        };
    }
    // *************************************************************************
    // PhysicsCollisionListener methods

    /**
     * Typically invoked 2x for each collision that happens in the PhysicsSpace.
     *
     * @param event the event that occurred (not null, reusable)
     */
    @Override
    public void collision(PhysicsCollisionEvent event) {
        if (fixedPco != null) {
            testReport();
            terminateTest();
        }
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just after the physics has been stepped. A
     * good time to re-activate deactivated objects.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the time per physics step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        // do nothing
    }

    /**
     * Callback from Bullet, invoked just before the physics is stepped. A good
     * time to clear/apply forces and reposition kinematic objects.
     *
     * @param space the space that is about to be stepped (not null)
     * @param timeStep the time per physics step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        if (overlapDistance < 0f) { // overlap not yet detected
            int numLaps = 0;
            if (fixedPco instanceof PhysicsGhostObject) {
                PhysicsGhostObject ghost = (PhysicsGhostObject) fixedPco;
                numLaps += ghost.getOverlappingObjects().size();
            }
            if (movingPco instanceof PhysicsGhostObject) {
                PhysicsGhostObject ghost = (PhysicsGhostObject) movingPco;
                numLaps += ghost.getOverlappingObjects().size();
            }
            if (numLaps > 0) {
                overlapDistance = distanceChebyshev();
            }
        }

        if (NativeLibraryLoader.isUsingNativeBullet()) {
            if (movingPco != null
                    && movingPco.getCollisionShape() instanceof SphereCollisionShape
                    && fixedPco != null
                    && fixedPco.getCollisionShape() instanceof SphereCollisionShape) {
                /*
                 * Abort sphere/sphere tests with Native Bullet immediately.
                 * This is to avoid an infinite loop since Native Bullet
                 * doesn't report sphere/sphere collisions. (JME issue #1029)
                 */
                reportResults("ABORTED");
                terminateTest();
            }

        } else if (movingPco instanceof PhysicsGhostObject
                && fixedPco instanceof PhysicsGhostObject) {
            /*
             * Abort ghost/ghost tests with JBullet immediately. This is to
             * avoid an infinite loop since JBullet doesn't report overlaps
             * OR collisions between 2 ghosts.
             */
            reportResults("ABORTED");
            terminateTest();

        } else if ((movingPco != null
                && movingPco.getCollisionShape() instanceof SimplexCollisionShape)
                || (fixedPco != null
                && fixedPco.getCollisionShape() instanceof SimplexCollisionShape)) {
            /*
             * Abort simplex tests with JBullet immediately. This is to
             * avoid an infinite loop since JBullet doesn't properly implement
             * SimplexCollisionShape.
             */
            reportResults("ABORTED");
            terminateTest();
        }

        if (fixedPco == null && movingPco == null) {
            startNextTest();
            nextTest();
        } else {
            Vector3f b = location(movingPco);
            b.multLocal(0.9995f); // move the centers 0.05% closer
            setLocation(b);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Calculate a vector's Chebyshev length. For determining whether 2
     * axis-aligned bounding boxes intersect, this is a more useful metric than
     * the Eucliean length.
     *
     * @param vector the vector to measure (not null, unaffected)
     * @return the Chebyshev length (&ge;0)
     */
    private float chebyshev(Vector3f vector) {
        float x = Math.abs(vector.x);
        float y = Math.abs(vector.y);
        float z = Math.abs(vector.z);
        float result = Math.max(x, Math.max(y, z));
        return result;
    }

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setEnabled(false);

        float aspectRatio = ((float) cam.getWidth()) / cam.getHeight();
        cam.setFrustumTop(3f * size);
        cam.setFrustumBottom(-3f * size);
        cam.setFrustumLeft(-3f * aspectRatio * size);
        cam.setFrustumRight(3f * aspectRatio * size);
        cam.setParallelProjection(true);
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        BulletAppState bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        bulletAppState.setDebugEnabled(true);

        physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.setAccuracy(0.01f); // 10-msec timestep
        physicsSpace.addCollisionListener(this);
        physicsSpace.addTickListener(this);
        physicsSpace.setGravity(new Vector3f(0f, 0f, 0f));
        physicsSpace.setSolverNumIterations(15);
    }

    /**
     * Generate a short description of the specified object's shape.
     *
     * @param pco (not null, unaffected)
     * @return descriptive text (not null, not empty)
     */
    private String describeShape(PhysicsCollisionObject pco) {
        String description = pco.getCollisionShape().getClass().getSimpleName();
        description = description.replace("CollisionShape", "");

        return description;
    }

    /**
     * Generate a short description of the specified object's type.
     *
     * @param pco (not null, unaffected)
     * @return descriptive text (not null, not empty)
     */
    private String describeType(PhysicsCollisionObject pco) {
        String description = pco.getClass().getSimpleName();
        description = description.replace("Physics", "");
        description = description.replace("Object", "");
        description = description.replace("Body", "");

        return description;
    }

    /**
     * Calculate the Chebyshev distance between the 2 centers of collision
     * objects.
     *
     * @return the Chebyshev distance (&ge;0)
     */
    private float distanceChebyshev() {
        Vector3f fixedLocation = location(fixedPco);
        Vector3f movingLocation = location(movingPco);
        Vector3f offset = movingLocation.subtract(fixedLocation);
        float result = chebyshev(offset);

        return result;
    }

    /**
     * Initialize the location of the moving object.
     *
     * @param location the desired location vector (in physics-space
     * coordinates, not null)
     */
    private void initializeLocation(Vector3f location) {
        if (movingPco instanceof PhysicsGhostObject) {
            ((PhysicsGhostObject) movingPco).setPhysicsLocation(location);
        } else {
            PhysicsRigidBody rigid = (PhysicsRigidBody) movingPco;
            rigid.setPhysicsLocation(location);
        }
    }

    /**
     * Determine the center of the specified collision object.
     *
     * @param pco (not null, unaffected)
     * @return a location vector (in physics-space coordinates, not null)
     */
    private Vector3f location(PhysicsCollisionObject pco) {
        if (pco instanceof PhysicsGhostObject) {
            return ((PhysicsGhostObject) pco).getPhysicsLocation(null);
        } else {
            return ((PhysicsRigidBody) pco).getPhysicsLocation();
        }
    }

    /**
     * Reconfigure the 5 indices for the next test.
     */
    private void nextTest() {
        ++nextMovingTypeIndex;
        if (nextMovingTypeIndex >= 2) {
            nextMovingTypeIndex = 0;

            ++nextFixedTypeIndex;
            if (nextFixedTypeIndex >= 2) {
                nextFixedTypeIndex = 0;

                ++nextDirectionIndex;
                if (nextDirectionIndex >= directions.length) {
                    nextDirectionIndex = 0;

                    ++nextFixedShapeIndex;
                    if (nextFixedShapeIndex >= shapes.length) {
                        nextFixedShapeIndex = 0;

                        ++nextMovingShapeIndex;
                        if (nextMovingShapeIndex >= shapes.length) {
                            nextMovingShapeIndex = 0;
                            allTestsComplete = true;
                        }
                    }
                }
            }
        }
    }

    /**
     * Report results from the current test.
     *
     * @param results the text to report (not null)
     */
    private void reportResults(String results) {
        System.out.printf("size=%.1f %s/%s %s %s/%s: ", size,
                describeShape(movingPco),
                describeShape(fixedPco),
                describeDirectionOfMotion,
                describeType(movingPco),
                describeType(fixedPco));
        System.out.println(results);

        if (nextDirectionIndex == 0
                && nextFixedTypeIndex == 0
                && nextMovingTypeIndex == 0) {
            System.out.println();
        }
    }

    /**
     * Start a new test using the next set of indices.
     */
    private void startNextTest() {
        assert fixedPco == null;
        assert movingPco == null;

        CollisionShape fixedShape = shapes[nextFixedShapeIndex];
        if (nextFixedTypeIndex == 1) {
            fixedPco = new PhysicsGhostObject(fixedShape);
        } else {
            fixedPco = new PhysicsRigidBody(fixedShape, 0f);
        }
        physicsSpace.add(fixedPco);

        CollisionShape movingShape = shapes[nextMovingShapeIndex];
        if (nextMovingTypeIndex == 1) {
            movingPco = new PhysicsGhostObject(movingShape);
        } else {
            PhysicsRigidBody body = new PhysicsRigidBody(movingShape, 1f);
            body.setLinearSleepingThreshold(0.001f * size);
            movingPco = body;
        }
        physicsSpace.add(movingPco);

        Vector3f directionOfMotion = directions[nextDirectionIndex];
        assert directionOfMotion.isUnitVector() : directionOfMotion;
        describeDirectionOfMotion
                = String.format("dir[%d]", nextDirectionIndex);
        /*
         * Start with the bounding boxes separated by 40% of a half extent.
         */
        float initDistance = 2.4f * size / chebyshev(directionOfMotion);
        initializeLocation(directionOfMotion.mult(-initDistance));
    }

    /**
     * Alter the location of the moving object.
     */
    private void setLocation(Vector3f location) {
        if (movingPco instanceof PhysicsGhostObject) {
            ((PhysicsGhostObject) movingPco).setPhysicsLocation(location);
        } else {
            PhysicsRigidBody rigid = (PhysicsRigidBody) movingPco;
            /*
             * Altering a the body's location directly would disable
             * collision detection, so apply impulses instead.
             */
            float timeStep = physicsSpace.getAccuracy();
            Vector3f motion = rigid.getLinearVelocity().mult(timeStep);
            Vector3f expectedLocation = rigid.getPhysicsLocation().add(motion);
            Vector3f locationError = location.subtract(expectedLocation);
            Vector3f deltaV = locationError.divide(timeStep);
            Vector3f impulse = deltaV.mult(rigid.getMass());
            rigid.applyImpulse(impulse, translateIdentity);
        }
    }

    /**
     * Terminate the current test.
     */
    private void terminateTest() {
        assert fixedPco != null;
        assert movingPco != null;

        physicsSpace.remove(fixedPco);
        fixedPco = null;
        physicsSpace.remove(movingPco);
        movingPco = null;
        overlapDistance = -1f;
    }

    /**
     * Report results of the current test.
     */
    private void testReport() {
        String results = "";
        if (overlapDistance >= 0f) {
            results += String.format("overlap at %.3f, ", overlapDistance);
        }
        float collisionDistance = distanceChebyshev();
        results += String.format("collision at %.3f", collisionDistance);
        reportResults(results);

        if (allTestsComplete) {
            stop();
        }
    }
}
