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

import com.jme3.app.SimpleApplication;
import com.jme3.app.StatsAppState;
import com.jme3.app.state.AppState;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.font.BitmapFont;
import com.jme3.font.BitmapText;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import java.util.Map;
import java.util.TreeMap;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Run performance tests and manage the results.
 */
public class TestRunner
        extends SimpleApplication
        implements TestListener {
    // *************************************************************************
    // constants and loggers

    /**
     * maximum percentage of lagging steps for a successful result
     */
    final private float maxPercentLag = 1f;
    /**
     * duration of each Measuring phase (in seconds)
     */
    final private float measuringDuration = 300f;
    /**
     * duration of each Stabilizing phase (in seconds)
     */
    final private float stabilizingDuration = 60f;
    /**
     * time-step for physics simulation (in seconds)
     */
    final private float timeStep = 1f / 60;
    /**
     * type of shape used in test
     */
    final private int shapeId = 2;
    /**
     * amount of load to use in the 1st test (depends on shapeId)
     */
    private int startingLoad = -1;
    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(TestRunner.class.getName());
    // *************************************************************************
    // fields

    /**
     * scene-graph node for displaying user-interface text
     */
    private BitmapText uiText;
    /**
     * map load levels to measured performance statistics
     */
    final private Map<Integer, PerfData> results = new TreeMap<>();
    /**
     * app state doing the measurements
     */
    private RigidBodyTestState testState;
    /**
     * string builder for the user interface
     */
    final private StringBuilder builder = new StringBuilder(100);
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        logger.setLevel(Level.INFO);
        TestRunner application = new TestRunner();
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
        configureUi();

        CollisionShape gemShape;
        float addRemoveRate;
        switch (shapeId) {
            case 1:
                gemShape = new SphereCollisionShape(0.1f);
                addRemoveRate = 10f;
                startingLoad = 400;
                break;

            case 2:
                gemShape
                        = new BoxCollisionShape(new Vector3f(0.1f, 0.1f, 0.1f));
                addRemoveRate = 5f;
                startingLoad = 200;
                break;

            case 3:
                Geometry teapot = (Geometry) assetManager.loadModel(
                        "Models/Teapot/Teapot.obj");
                gemShape = new HullCollisionShape(teapot.getMesh());
                gemShape.setScale(new Vector3f(0.5f, 0.5f, 0.5f));
                addRemoveRate = 1f;
                startingLoad = 10;
                break;

            default:
                throw new RuntimeException(Integer.toString(shapeId));
        }
        gemShape.setMargin(0.005f);

        testState = new RigidBodyTestState(gemShape, timeStep, addRemoveRate,
                this);
        stateManager.attach(testState);
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        super.simpleUpdate(tpf);
        updateUi();
    }
    // *************************************************************************
    // TestListener methods

    /**
     * Callback when a test completes.
     *
     * @param appState the app state that completed the test (not null)
     */
    @Override
    public void complete(AppState appState) {
        assert appState == testState;

        int prevLoad = testState.targetLoad();
        PerfData data = testState.getResults();
        results.put(prevLoad, data);
        int nextLoad = nextLoad();
        /*
         * Time-per-step data is collected in 1-msec bins,
         * ranging from 10 msec to 50 msec.
         */
        PerfData tpsData = new PerfData(0.01f, 0.001f, 42);
        testState.measureLatency(nextLoad, stabilizingDuration,
                measuringDuration, tpsData);
    }

    /**
     * Callback when ready for the 1st test.
     *
     * @param appState the app state that's ready (not null)
     */
    @Override
    public void ready(AppState appState) {
        logger.log(Level.INFO, "");
        assert appState == testState;
        /*
         * Time-per-step data is collected in 1-msec bins,
         * ranging from 10 msec to 50 msec.
         */
        PerfData tpsData = new PerfData(0.01f, 0.001f, 42);
        testState.measureLatency(startingLoad, stabilizingDuration,
                measuringDuration, tpsData);
    }
    // *************************************************************************
    // private methods

    /**
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setEnabled(false);
        cam.setLocation(new Vector3f(3.4f, 1.6f, -4.3f));
        cam.setRotation(new Quaternion(0.076f, -0.3088f, -0.025f, 0.95777f));
    }

    /*
     * Add a BitmapText in the upper-left corner of the display.
     */
    private void configureUi() {
        BitmapFont font = assetManager.loadFont("Interface/Fonts/Default.fnt");
        uiText = new BitmapText(font);
        guiNode.attachChild(uiText);
        float displayHeight = cam.getHeight();
        uiText.move(0f, displayHeight, 0f);

        StatsAppState sas = stateManager.getState(StatsAppState.class);
        sas.setDisplayStatView(false);
    }

    /**
     * Select the next load level using a bisection search.
     */
    private int nextLoad() {
        int maxPass = 0; // highest load passed
        int minFail = Integer.MAX_VALUE; // lowest load failed
        for (Map.Entry<Integer, PerfData> entry : results.entrySet()) {
            PerfData stats = entry.getValue();
            float percentLag = stats.percentLastBin();
            float mean = stats.mean();
            boolean pass = (percentLag < maxPercentLag) && (mean < timeStep);

            int load = entry.getKey();
            if (pass && load > maxPass) {
                maxPass = load;
            }
            boolean fail = !pass;
            if (fail && load < minFail) {
                minFail = load;
            }
        }

        int nextLoad = 0;
        if (minFail == Integer.MAX_VALUE) {
            // increase load by 25%
            nextLoad = (int) FastMath.ceil(1.25f * maxPass);
            String desc = testState.describeWorkload(nextLoad);
            logger.log(Level.INFO, "No fail yet. Try {0}.", desc);

        } else if (maxPass == 0) {
            // reduce load by 20%
            nextLoad = (int) FastMath.floor(0.8f * minFail);
            assert nextLoad > 0 : nextLoad;
            String desc = testState.describeWorkload(nextLoad);
            logger.log(Level.INFO, "No pass yet. Try {0}.", desc);

        } else if (maxPass + 1 < minFail) {
            nextLoad = Math.round(0.5f * (maxPass + minFail));
            assert nextLoad > maxPass;
            assert nextLoad < minFail;
            logger.log(Level.INFO,
                    "Passed {0} but not {1}. Bisect the gap {2}.",
                    new Object[]{
                        testState.describeWorkload(maxPass),
                        testState.describeWorkload(minFail),
                        testState.describeWorkload(nextLoad)
                    });

        } else {
            // Print the overall result and terminate the application.
            logger.log(Level.SEVERE, "Overall result: passed {0}",
                    testState.describeWorkload(maxPass));
            stop();
        }

        return nextLoad;
    }

    /*
     * Update the user interface.
     */
    private void updateUi() {
        builder.delete(0, builder.length());

        int numResults = results.size();
        builder.append(numResults);
        builder.append(" result");
        if (numResults != 1) {
            builder.append("s");
        }
        builder.append(". ");

        TestPhase testPhase = testState.testPhase();
        int currentLoad = testState.currentLoad();
        builder.append(testPhase);
        if (testPhase == TestPhase.Ramping) {
            int targetLoad = testState.targetLoad();
            if (targetLoad > currentLoad) {
                builder.append(" up to ");
            } else {
                builder.append(" down to ");
            }
            builder.append(targetLoad);
        }
        builder.append(" ");
        String desc = testState.describeWorkload(currentLoad);
        builder.append(desc);
        builder.append(".");

        if (testPhase != null && testPhase != TestPhase.Ramping) {
            double phaseTimer = testState.timeRemainingInPhase();
            builder.append(" ");
            int numSeconds = (int) Math.round(phaseTimer);
            builder.append(numSeconds);
            builder.append(" second");
            if (numSeconds != 1) {
                builder.append("s");
            }
            builder.append(" to go.");
        }

        uiText.setText(builder);
    }
}
