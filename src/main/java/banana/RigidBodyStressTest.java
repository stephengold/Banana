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
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.font.BitmapFont;
import com.jme3.font.BitmapText;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import java.util.Random;
import java.util.logging.Logger;

/**
 * Determine how many closely-packed rigid bodies Bullet Physics can support at
 * 15 fps.
 *
 * Run with Vsync enabled!
 */
public class RigidBodyStressTest extends SimpleApplication {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RigidBodyStressTest.class.getName());
    // *************************************************************************
    // fields

    /**
     * scene-graph node for displaying user-interface text
     */
    private BitmapText uiText;
    /**
     * true for one frame
     */
    private boolean isFirstFrame = true;
    /**
     * shape for falling gems
     */
    private CollisionShape gemShape;
    /**
     * accumulate tpf up to 1 second
     */
    private float secondCounter = 0f;
    /**
     * count the frames in one second
     */
    private int frameCounter = 0;
    /**
     * number of falling bodies in the scene
     */
    private int numGems = 0;
    /**
     * physics space
     */
    private PhysicsSpace physicsSpace;
    /**
     * pseudo-random generator
     */
    final private Random random = new Random(1L);
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        RigidBodyStressTest application = new RigidBodyStressTest();
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
        configurePhysics();
        addBoxes();

        int shapeId = 3;
        switch (shapeId) {
            case 1:
                gemShape = new SphereCollisionShape(0.1f);
                break;
            case 2:
                gemShape
                        = new BoxCollisionShape(new Vector3f(0.1f, 0.1f, 0.1f));
                break;
            case 3:
                Geometry teapot = (Geometry) assetManager.loadModel(
                        "Models/Teapot/Teapot.obj");
                gemShape = new HullCollisionShape(teapot.getMesh());
                gemShape.setScale(new Vector3f(0.5f, 0.5f, 0.5f));
        }

        gemShape.setMargin(0.005f);
    }

    /**
     * Callback invoked once per frame.
     *
     * @param tpf time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void simpleUpdate(float tpf) {
        super.simpleUpdate(tpf);

        if (isFirstFrame) {
            // The first frame includes startup time, so ignore it.
            isFirstFrame = false;
        } else {
            secondCounter += getTimer().getTimePerFrame();
        }
        /*
         * Calculate the frame rate and abort the test if it's too low.
         */
        frameCounter++;
        if (secondCounter >= 1f) {
            float fps = frameCounter / secondCounter;
            if (fps < 15f) {
                System.out.printf("final numGems = %d%n", numGems);
                stop();
            }
            secondCounter = 0f;
            frameCounter = 0;
        }
        /*
         * Add complex shapes once per second, simple shapes once per frame.
         */
        if (!(gemShape instanceof HullCollisionShape) || frameCounter == 0) {
            addAGem();
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Add a falling PhysicsRigidBody to the scene.
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

        physicsSpace.add(body);
        body.setGravity(new Vector3f(0f, -9f, 0f));

        ++numGems;
        /*
         * Update the user interface.
         */
        String msg = String.format("numGems=%d", numGems);
        uiText.setText(msg);
    }

    /**
     * Add 3 large, static boxes to catch the falling bodies.
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
        rootNode.attachChild(boxes);
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
     * Configure the camera during startup.
     */
    private void configureCamera() {
        flyCam.setEnabled(false);
        cam.setLocation(new Vector3f(3.4f, 1.6f, -4.3f));
        cam.setRotation(new Quaternion(0.076f, -0.3088f, -0.025f, 0.95777f));
    }

    /**
     * Configure physics during startup.
     */
    private void configurePhysics() {
        BulletAppState bulletAppState = new BulletAppState();
        //bulletAppState.setDebugEnabled(true);
        bulletAppState.setThreadingType(BulletAppState.ThreadingType.PARALLEL);
        stateManager.attach(bulletAppState);

        physicsSpace = bulletAppState.getPhysicsSpace();
        physicsSpace.setAccuracy(1f / 60); // 16.67 msec timestep
        physicsSpace.setSolverNumIterations(10);
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
    }

    /**
     * Generate an orthonormal basis.
     *
     * @param in1 input direction for 1st basis vector (not null, not zero,
     * modified)
     * @param store2 storage for the 2nd basis vector (not null, modified)
     * @param store3 storage for the 3nd basis vector (not null, modified)
     */
    private void generateBasis(Vector3f in1, Vector3f store2, Vector3f store3) {
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
}
