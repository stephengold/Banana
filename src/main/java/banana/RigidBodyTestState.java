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
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * An AppState to measure physics performance with closely-packed rigid bodies.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class RigidBodyTestState
        extends AbstractAppState
        implements PhysicsTickListener {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(RigidBodyTestState.class.getName());
    // *************************************************************************
    // fields

    /**
     * manager for app states
     */
    private AppStateManager stateManager;
    /**
     * app state to manage the PhysicsSpace.
     */
    private BulletAppState bulletAppState;
    /**
     * physics shape for dynamic bodies
     */
    final private CollisionShape gemShape;
    /**
     * time until the end of the current phase (in seconds)
     */
    private double phaseTimer = 0.0;
    /**
     * stable rate of add-remove operations (in Hertz)
     */
    final private float addRemoveRate;
    /**
     * duration of each Measuring phase (in seconds)
     */
    private float measuringDuration;
    /**
     * duration of each Stabilizing phase (in seconds)
     */
    private float stabilizingDuration;
    /**
     * time-step for physics simulation (in seconds)
     */
    final private float timeStep;
    /**
     * desired number of dynamic bodies for the Measuring phase
     */
    private int targetLoad;
    /**
     * nanoTime for the next add-remove action (in nanoseconds)
     */
    private long addRemoveTime = 0L;
    /**
     * nanoTime at the start of the most-recent physics step (in nanoseconds)
     */
    private long preTime = 0L;
    /**
     * current time-per-step statistics
     */
    private PerfData tpsData;
    /**
     * dynamic bodies in the physics space, on a first-in/first-out basis
     */
    final private Queue<PhysicsRigidBody> gems = new LinkedList<>();
    /**
     * pseudo-random generator
     */
    final private Random random = new Random(1L);
    /**
     * description of the current workload
     */
    private String currentDescription;
    /**
     * listener to notify when ready to run a new test
     */
    final private TestListener listener;
    /**
     * which test phase is running
     */
    private TestPhase testPhase = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a new test state.
     *
     * @param gemShape (not null)
     * @param timeStep (&gt;0)
     * @param addRemoveRate (&gt;0)
     * @param listener the listener to notify when the current test completes
     * (not null)
     */
    RigidBodyTestState(CollisionShape gemShape, float timeStep,
            float addRemoveRate, TestListener listener) {
        assert gemShape != null;
        assert timeStep > 0f : timeStep;
        assert addRemoveRate > 0f : addRemoveRate;
        assert listener != null;

        this.gemShape = gemShape;
        this.timeStep = timeStep;
        this.addRemoveRate = addRemoveRate;
        this.listener = listener;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the current load level.
     *
     * @return the number of dynamic rigid bodies (&ge;0)
     */
    int currentLoad() {
        return gems.size();
    }

    /**
     * Describe a workload.
     *
     * @param the number of dynamic bodies (&ge;0)
     * @return a textual description (not null, not empty)
     */
    String describeWorkload(int numGems) {
        String description;
        if (gemShape instanceof BoxCollisionShape) {
            description = String.format("with %d box%s", numGems,
                    numGems == 1 ? "" : "es");
        } else if (gemShape instanceof HullCollisionShape) {
            description = String.format("with %d hull%s", numGems,
                    numGems == 1 ? "" : "s");
        } else if (gemShape instanceof SphereCollisionShape) {
            description = String.format("with %d sphere%s", numGems,
                    numGems == 1 ? "" : "s");
        } else {
            throw new IllegalStateException(gemShape.getClass().getName());
        }

        return description;
    }

    /**
     * Access the measurement data.
     *
     * @return the pre-existing instance (not null)
     */
    PerfData getResults() {
        return tpsData;
    }

    /**
     * Measure physics step latency at the specified load level.
     *
     * @param loadLevel the number of dynamic rigid bodies (&ge;0)
     * @param stabilizingDuration (in seconds, &ge;0)
     * @param measuringDuration (in seconds, &gt;0)
     * @param tpsData storage for measured TPS data (not null)
     */
    void measureLatency(int loadLevel, float stabilizingDuration,
            float measuringDuration, PerfData tpsData) {
        assert loadLevel >= 0 : loadLevel;
        assert stabilizingDuration >= 0f : stabilizingDuration;
        assert measuringDuration > 0f : measuringDuration;
        assert tpsData != null;
        assert testPhase == null;

        this.targetLoad = loadLevel;
        this.stabilizingDuration = stabilizingDuration;
        this.measuringDuration = measuringDuration;
        this.tpsData = tpsData;
        /*
         * Start ramping up the workload.
         */
        testPhase = TestPhase.Ramping;
    }

    /**
     * Read the target load level.
     *
     * @return the desired number of dynamic rigid bodies (&ge;0)
     */
    int targetLoad() {
        assert targetLoad >= 0 : targetLoad;
        return targetLoad;
    }

    /**
     * Read which phase the running test is in.
     *
     * @return an enum value, or null if no test is running
     */
    TestPhase testPhase() {
        return testPhase;
    }

    /**
     * Read the amount of time remaining in the current test phase. Valid only
     * during the Stabilizing and Measuring phases.
     *
     * @return a time interval (in seconds, &ge;0)
     */
    double timeRemainingInPhase() {
        assert testPhase != null;
        assert testPhase != TestPhase.Ramping;
        assert phaseTimer >= 0 : phaseTimer;
        return phaseTimer;
    }
    // *************************************************************************
    // AbstractAppState methods

    /**
     * Transition this state from terminating to detached. Should be invoked
     * only by a subclass or by the AppStateManager. Invoked once for each time
     * {@link #initialize(com.jme3.app.state.AppStateManager, com.jme3.app.Application)}
     * was invoked.
     */
    @Override
    public void cleanup() {
        stateManager.detach(bulletAppState);
    }

    /**
     * Initialize this state prior to its 1st update. Should be invoked only by
     * a subclass or by the AppStateManager.
     *
     * @param stateManager the manager for this state (not null)
     * @param app the application which owns this state (not null)
     */
    @Override
    public void initialize(AppStateManager stateManager, Application app) {
        super.initialize(stateManager, app);

        this.stateManager = stateManager;

        bulletAppState = new BulletAppState();
        //bulletAppState.setDebugEnabled(true);
        bulletAppState.setThreadingType(BulletAppState.ThreadingType.PARALLEL);
        stateManager.attach(bulletAppState);

        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.addTickListener(this);
        physicsSpace.setAccuracy(timeStep);
        physicsSpace.setMaxSubSteps(4);
        physicsSpace.setSolverNumIterations(10);

        addBoxes();
        listener.ready(this);
    }

    /**
     * Update this state prior to rendering. Should be invoked only by a
     * subclass or by the AppStateManager. Invoked once per frame, provided the
     * state is attached and enabled.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        if (testPhase == null) {
            return;
        }

        int currentLoad = gems.size();

        if (testPhase == TestPhase.Ramping) {
            if (currentLoad == targetLoad) {
                startStabilizing();
            }
            return;
        }

        assert currentLoad == targetLoad;
        /*
         * Update the phase timer.
         */
        phaseTimer -= tpf;
        if (phaseTimer > 0.0) {
            return;
        }
        /*
         * The timer has run out, so a test phase has completed.
         */
        switch (testPhase) {
            case Stabilizing:
                startMeasuring();
                break;
            case Measuring:
                stopMeasuring();
                break;
            default:
                throw new IllegalStateException(testPhase.toString());
        }
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback from Bullet, invoked just after the physics has been stepped.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the time per physics step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        long elapsed = System.nanoTime() - preTime;
        if (tpsData != null) {
            tpsData.add(1e-9f * elapsed);
        }
    }

    /**
     * Callback from Bullet, invoked just before the physics is stepped.
     *
     * @param space the space that is about to be stepped (not null)
     * @param timeStep the time per physics step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        preTime = System.nanoTime();

        if (addRemoveTime < preTime) {
            /*
             * It's time to add & remove dynamic bodies.
             */
            if (testPhase == TestPhase.Ramping) {
                int currentLoad = gems.size();
                if (targetLoad > currentLoad) {
                    addAGem();
                } else if (targetLoad < currentLoad) {
                    removeOldestGem();
                }
            } else { // Stabilizing or Measuring
                addAGem();
                removeOldestGem();
            }
            addRemoveTime = preTime + Math.round(1e9 / addRemoveRate);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Add a dynamic PhysicsRigidBody to the PhysicsSpace.
     */
    private void addAGem() {
        float x = 2f * random.nextFloat() - 1f;
        float y = 2f * random.nextFloat() - 1f;
        float z = 2f * random.nextFloat() - 1f;
        Vector3f startLocation = new Vector3f(x, y, z);
        startLocation.multLocal(0.5f, 1f, 0.5f);
        startLocation.y += 4f;

        float mass = 1f;
        PhysicsRigidBody body = new PhysicsRigidBody(gemShape, mass);
        body.setDamping(0.6f, 0.6f);
        body.setFriction(1f);
        body.setKinematic(false);
        body.setPhysicsLocation(startLocation);

        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.add(body);
        body.setGravity(new Vector3f(0f, -9f, 0f));

        gems.add(body);
        currentDescription = describeWorkload(gems.size());
    }

    /**
     * Add 3 large, static boxes to the PhysicsSpace, for catching falling
     * bodies.
     */
    private void addBoxes() {
        /*
         * Calculate a quaternion to rotate (1,1,1) to the X axis.
         */
        Vector3f v1 = new Vector3f(1f, 1f, 1f);
        Vector3f v2 = new Vector3f();
        Vector3f v3 = new Vector3f();
        generateBasis(v1, v2, v3);
        Quaternion rotate111 = new Quaternion();
        rotate111.fromAxes(v1, v2, v3);

        Node boxes = new Node("boxes");
        boxes.rotate(rotate111);

        float halfExtent = 50f;
        Vector3f hes = new Vector3f(halfExtent, halfExtent, halfExtent);
        BoxCollisionShape boxShape = new BoxCollisionShape(hes);
        float mass = 0f;

        Node box1 = new Node("box1");
        boxes.attachChild(box1);
        box1.setLocalTranslation(-halfExtent, halfExtent, halfExtent);
        RigidBodyControl rbc1 = new RigidBodyControl(boxShape, mass);
        box1.addControl(rbc1);
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
        rbc1.setPhysicsSpace(physicsSpace);

        Node box2 = new Node("box2");
        boxes.attachChild(box2);
        box2.setLocalTranslation(halfExtent, -halfExtent, halfExtent);
        RigidBodyControl rbc2 = new RigidBodyControl(boxShape, mass);
        box2.addControl(rbc2);
        rbc2.setPhysicsSpace(physicsSpace);

        Node box3 = new Node("box3");
        boxes.attachChild(box3);
        box3.setLocalTranslation(halfExtent, halfExtent, -halfExtent);
        RigidBodyControl rbc3 = new RigidBodyControl(boxShape, mass);
        box3.addControl(rbc3);
        rbc3.setPhysicsSpace(physicsSpace);
    }

    /**
     * Generate an orthonormal basis that includes the specified vector.
     *
     * @param in1 input direction for 1st basis vector (not null, not zero,
     * modified)
     * @param store2 storage for the 2nd basis vector (not null, modified)
     * @param store3 storage for the 3nd basis vector (not null, modified)
     */
    private static void generateBasis(Vector3f in1, Vector3f store2,
            Vector3f store3) {
        in1.normalizeLocal();
        /*
         * Pick a direction that's not parallel (or anti-parallel) to
         * the input direction.
         */
        float x = Math.abs(in1.x);
        float y = Math.abs(in1.y);
        float z = Math.abs(in1.z);
        if (x <= y && x <= z) {
            store3.set(1f, 0f, 0f);
        } else if (y <= z) {
            store3.set(0f, 1f, 0f);
        } else {
            store3.set(0f, 0f, 1f);
        }
        /*
         * Use cross products to generate unit vectors orthogonal
         * to the input vector.
         */
        in1.cross(store3, store2);
        store2.normalizeLocal();
        in1.cross(store2, store3);
        store3.normalizeLocal();
    }

    /**
     * Remove the oldest dynamic PhysicsRigidBody from the PhysicsSpace.
     */
    private void removeOldestGem() {
        PhysicsRigidBody gem = gems.remove();
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.remove(gem);
        currentDescription = describeWorkload(gems.size());
    }

    /**
     * Start a Measuring phase.
     */
    private void startMeasuring() {
        // Begin the Measuring phase.
        testPhase = TestPhase.Measuring;
        phaseTimer = measuringDuration;
        logger.log(Level.INFO, "Measure for {0} sec {1}.",
                new Object[]{
                    phaseTimer, describeWorkload(targetLoad)
                });
        tpsData.clear();
    }

    /**
     * Start a Stabilizing phase.
     */
    private void startStabilizing() {
        testPhase = TestPhase.Stabilizing;
        phaseTimer = stabilizingDuration;
        logger.log(Level.INFO, "Stabilize for {0} sec {1}.",
                new Object[]{
                    phaseTimer, describeWorkload(targetLoad)
                });
    }

    /**
     * Stop a Measuring phase.
     */
    private void stopMeasuring() {
        testPhase = null;
        phaseTimer = 0.0;

        System.out.printf("Result %s:%n", currentDescription);
        tpsData.dump(System.out);

        listener.complete(this);
    }
}
