#include "raylib.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <numeric>

using namespace std;

// Setup
    // ✅ 1 - Window — opens, holds, closes cleanly. (60 FPS, dark background.)
    // ✅ 2 - Pivot point — draw the fixed anchor as a dot.
    // ✅ 3 - One rigid arm — pivot to a bob, at a fixed angle, no motion.
    // ✅ 4 - Swing it — single-pendulum physics on that one arm.
    // ✅ 5 - Second arm — add the lower arm hanging off the first.
    // ✅ 6 - True double pendulum — the real coupled equations, plus a motion trail.

// Pivot point
struct Pivot {
    float x;
    float y;
    float r;
    Color c;

    Pivot(float x, float y, float r, Color c) : x(x), y(y), r(r), c(c) {}

    void draw() {
        DrawCircle(x, y, r, c);
    }
};

// Arms
struct Arm {
    Vector2 start;
    float angle;            // radians, measured from horizontal (+x). REAL state.
    float length;
    Color c;
    int r = 10;             // bob radius

    float angularVelocity = 0.0f;   // how fast `angle` is changing
    float mass = 1.0f;

    Arm(Vector2 start, float angle, float length, Color c, int r = 10, float mass = 1.0f)
        : start(start), angle(angle), length(length), c(c), r(r), mass(mass){}

    // end point is DERIVED from angle+length — never stored, always in sync
    Vector2 end() {
        return { start.x + length * cos(angle),
                 start.y + length * sin(angle) };
    }

    void draw() {
        DrawLineV(start, end(), c);
        DrawCircleV(end(), 2*r, c);
    }
};

// NOTE: no longer used — we switched from "std-dev over a window" to "drift from
// initial energy", which needs no history. Kept for reference in case you want a
// short-term noisiness readout later.
/*
float get_sample_std_dev(const std::vector<float>& v) {
    if (v.size() <= 1) return 0.0;

    double mean = accumulate(v.begin(), v.end(), 0.0) / v.size();

    double sq_sum = inner_product(v.begin(), v.end(), v.begin(), 0.0,
        plus<>(),
        [mean](double a, double b) { return (a - mean) * (b - mean); }
    );

    return sqrt(sq_sum / (v.size() - 1)); // N - 1 for sample deviation
}
*/

// The complete state of the double pendulum: the four numbers that fully
// determine its future. theta values are measured FROM VERTICAL (the convention
// the physics equations are written in).
struct State {
    float theta1;    // arm 1 angle (from vertical)
    float omega1;    // arm 1 angular velocity
    float theta2;    // arm 2 angle (from vertical)
    float omega2;    // arm 2 angular velocity

    // State (float theta1, float omega1, float theta2, float omega2) : theta1(theta1), omega1(omega1), theta2(theta2), omega2(omega2) {}
};

// Given a state, return the RATE OF CHANGE of each component.
// State/velocity come from `s`; constants (mass, length) come from the arms; g passed in.
State derivatives(State s, Arm& a1, Arm& a2, float g) {
    State d;

    // The rate of change of an ANGLE is its angular velocity — by definition.
    d.theta1 = s.omega1;
    d.theta2 = s.omega2;

    // The rate of change of a VELOCITY is the angular acceleration (alpha) —
    // your coupled equations, now reading angles/velocities from `s`.
    float deltaAngle = s.theta1 - s.theta2;

    float num1 = -g * (2.0f * a1.mass + a2.mass) * sin(s.theta1)
                - a2.mass * g * sin(s.theta1 - 2.0f * s.theta2)
                - 2.0f * sin(deltaAngle) * a2.mass * (s.omega2 * s.omega2 * a2.length + s.omega1 * s.omega1 * a1.length * cos(deltaAngle));

    float den1 = a1.length * (2.0f * a1.mass + a2.mass - a2.mass * cos(2.0f * s.theta1 - 2.0f * s.theta2));

    d.omega1 = num1 / den1;   // = alpha1

    float num2 = 2.0f * sin(deltaAngle) * (s.omega1 * s.omega1 * a1.length * (a1.mass + a2.mass)
                + g * (a1.mass + a2.mass) * cos(s.theta1)
                + s.omega2 * s.omega2 * a2.length * a2.mass * cos(deltaAngle));

    float den2 = a2.length * (2.0f * a1.mass + a2.mass - a2.mass * cos(2.0f * s.theta1 - 2.0f * s.theta2));

    d.omega2 = num2 / den2;   // = alpha2

    return d;
}

// returns s + d*h  (step state s along derivative d by amount h)
State step(State s, State d, float h) {
    return { s.theta1 + d.theta1 * h,
             s.omega1 + d.omega1 * h,
             s.theta2 + d.theta2 * h,
             s.omega2 + d.omega2 * h };
}

// Reset button
struct button {

    string value;
    Color c = DARKGRAY;
    Color onhover = GRAY;

    void draw () {

        DrawRectangle(10, 115, 175, 30, c);

        DrawText(value.c_str(), 15, 120, 20, RAYWHITE);

    }

    void onhoverdraw () {
        DrawRectangle(10, 115, 175, 30, onhover);

        DrawText(value.c_str(), 15, 120, 20, RAYWHITE);
    }

};

// Program main entry point
int main(void)
{
    // Initialization
    const int screenWidth = 800;
    const int screenHeight = 400;

    InitWindow(screenWidth, screenHeight, "Double Pendulum");

    // Camera: shift origin (0,0) to the middle of the screen.
    // NOTE: pure translation — y still points DOWN. +y = down the screen.
    Camera2D camera = { 0 };
    camera.offset = (Vector2){ GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f };
    camera.target = (Vector2){ 0.0f, 0.0f };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    SetTargetFPS(60);

    // UI
    bool dragging = false;
    bool dragMid = false;
    bool dragEnd = false;

    // ---- Physics constants ----
    const float g = 800.0f;   // pixels/sec^2 — tuned by feel, not real 9.8

    // Pendulum setup
    Pivot p1 (0, 0, 20, RED);

    float length = 100.0f;
    float thetaInitial = 20.0f;

    Arm a1( {p1.x, p1.y},
            PI / 180.0f * thetaInitial,
            length,
            RAYWHITE );

    Arm a2( a1.end(),
            PI / 180.0f * thetaInitial,
            length,
            BLUE);

    vector<Vector2> trail = {a2.end()};
    const int TRAIL_MAX = 10000;

    // --- Energy tracking ---
    // NOTE: the energy-history vector + its cap are no longer needed. Drift only
    // needs the ONE initial value and the current value — no history to store.
    // const int ENERGY_MAX = 200;
    // vector<float> energy;

    float initialEnergy = 0.0f;   // captured ONCE on the first frame (see below)
    bool  firstFrame = true;      // guard so we grab initialEnergy exactly once
    float drift = 0.0f;

    // display throttling
    float displayTimer = 0.0f;
    float displayedEnergy = 0.0f;

    string E;
    string K;
    string P;
    string D;

    button trace {"Reset Trace"};
    Rectangle traceRect = { 10, 115, 175, 30 };
    bool traceHover = false;

    // Main game loop
    while (!WindowShouldClose())
    {
        // ================= UPDATE =================

        float dt = GetFrameTime();

        // --- Input: drag the pivot around ---
        Vector2 world = GetScreenToWorld2D(GetMousePosition(), camera);

        Vector2 screen = GetMousePosition();   // name it, for UI hit-tests

        traceHover = CheckCollisionPointRec(screen, traceRect);   // screen space

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (CheckCollisionPointCircle(world, { p1.x, p1.y }, p1.r)) {
                dragging = true;
            } else if (CheckCollisionPointCircle(world, a1.end(), a1.r)) {
                dragMid = true;
            } else if (CheckCollisionPointCircle(world, a2.end(), a2.r)) {   // world, not screen
                dragEnd = true;
            } else if (CheckCollisionPointRec(screen, traceRect)) {          // screen, and full rect
                trail = { a2.end() };                                        // reset the trace
            }
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            dragging = false;
            dragMid = false;
            dragEnd = false;

            // if we were dragging anything, the system's been repositioned —
            // re-baseline so drift measures conservation from HERE, not from launch
            firstFrame = true;   // reuse your existing "capture once" guard
        }
        if (dragging) {
            p1.x = world.x;
            p1.y = world.y;
            a1.start = { p1.x, p1.y };
            a2.start = a1.end();
        } else if (dragMid) {
            // Need to calculate distance from touch to a1 
            float dx = world.x - a1.start.x;
            float dy = world.y - a1.start.y;
            a1.angle = atan2(dy, dx);
            a2.start = a1.end();
        } else if (dragEnd) {
            // Need to calculate distance from touch to a2
            float dx = world.x - a2.start.x;
            float dy = world.y - a2.start.y;
            a2.angle = atan2(dy, dx);
            a2.angularVelocity = 0.0f;
        }
        float totalEnergy;
        float potentialEnergy;
        float kineticEnergy;
        // --- PENDULUM PHYSICS RK4---
        if (!dragging && !dragMid && !dragEnd) {
        
            State s {a1.angle-PI/2, a1.angularVelocity, a2.angle - PI/2, a2.angularVelocity};

            State k1 = derivatives(s, a1, a2, g);
            State k2 = derivatives(step(s, k1, dt/2.0f), a1, a2, g);
            State k3 = derivatives(step(s, k2, dt/2.0f), a1, a2, g);
            State k4 = derivatives(step(s, k3, dt), a1, a2, g);

            //s_next = s + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

            State s_next = step(s, step(step(step(k1, k2, 2.0f), k3, 2.0f), k4, 1.0f), dt/6.0f);

            a1.angle = s_next.theta1 + PI/2;
            a1.angularVelocity = s_next.omega1;
            a2.angle = s_next.theta2 + PI/2;
            a2.angularVelocity = s_next.omega2;

            a2.start = a1.end();

           
        }
        // NOTE: no longer accumulating an energy history.
        // energy.push_back(totalEnergy);

        // --- Energy: computed EVERY frame from the arms' current state ---
        // Works in both modes: RK4 writes the arms during physics; your finger writes
        // them during a drag. Either way the arms are the source of truth.
        float th1 = a1.angle - PI / 2.0f;   // convert to from-vertical for the physics formula
        float th2 = a2.angle - PI / 2.0f;
        float w1  = a1.angularVelocity;
        float w2  = a2.angularVelocity;
        float dTheta = th1 - th2;

        kineticEnergy = (0.5f * (a1.mass + a2.mass) * a1.length * a1.length * w1 * w1
                    + 0.5f * a2.mass * a2.length * a2.length * w2 * w2
                    + a2.mass * a1.length * a2.length * w1 * w2 * cos(dTheta)) / 10000.0f;

        potentialEnergy = (-g * (a1.mass + a2.mass) * a1.length * cos(th1)
                        - g * a2.mass * a2.length * cos(th2)) / 10000.0f;

        totalEnergy = kineticEnergy + potentialEnergy;

        if (firstFrame) {
            initialEnergy = totalEnergy;
            firstFrame = false;
        }

        // ================= DRAW =================
        BeginDrawing();
            ClearBackground(BLACK);
            BeginMode2D(camera);

                // faded, continuous trail (newer = brighter)
                for (int i = 1; i < trail.size(); i++) {
                    float alpha = (float)i / trail.size();
                    DrawLineV(trail[i-1], trail[i], Fade(WHITE, alpha));
                }

                p1.draw();
                a1.draw();
                a2.draw();

            EndMode2D();

            // --- throttled text update (every 0.2s) ---
            displayTimer += dt;
            if (displayTimer >= 0.2f) {
                displayedEnergy = totalEnergy;

                drift = (totalEnergy - initialEnergy) / fabs(initialEnergy) * 100.0f;

                // E uses the (now-disabled) std-dev; simplified to plain total energy.
                // E = "Total Energy: " + to_string(round(totalEnergy * 100.0) / 100.0) + " ± " + to_string(round(get_sample_std_dev(energy) * 100.0) / 100.0);
                E = "Total Energy: " + to_string(round(totalEnergy * 100.0) / 100.0);
                K = "Kinetic Energy: " + to_string(round(kineticEnergy * 100.0) / 100.0);
                P = "Potential Energy: " + to_string(round(potentialEnergy * 100.0) / 100.0);
                D = "Energy Drift: " + to_string(round(drift * 100.0) / 100.0) + "%";

                displayTimer = 0.0f;
            }

            DrawText(E.c_str(), 10, 10, 20, WHITE);
            DrawText(K.c_str(), 10, 35, 20, WHITE);
            DrawText(P.c_str(), 10, 60, 20, WHITE);
            DrawText(D.c_str(), 10, 85, 20, WHITE);

            if (traceHover) {
                trace.onhoverdraw();
            } else {
                trace.draw();
            }

        EndDrawing();

        // --- trail bookkeeping ---
        trail.push_back(a2.end());
        if (trail.size() > TRAIL_MAX) {
            trail.erase(trail.begin());
        }

        // NOTE: energy history cap no longer needed (vector removed).
        // if (energy.size() > ENERGY_MAX) {
        //     energy.erase(energy.begin());
        // }
    }

    CloseWindow();
    return 0;
}