#include "raylib.h"
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// Setup
    // ✅ 1 - Window — opens, holds, closes cleanly. (60 FPS, dark background.)
    // ✅ 2 - Pivot point — draw the fixed anchor as a dot.
    // ✅ 3 - One rigid arm — pivot to a bob, at a fixed angle, no motion.
    // 4 - Swing it — single-pendulum physics on that one arm.   ← THIS STEP
    // 5 - Second arm — add the lower arm hanging off the first.
    // 6 - True double pendulum — the real coupled equations, plus a motion trail.

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

    // NEW physics state:
    float angularVelocity = 0.0f;   // how fast `angle` is changing (rad/frame-ish)

    Arm(Vector2 start, float angle, float length, Color c, int r = 10)
        : start(start), angle(angle), length(length), c(c), r(r) {}

    // end point is DERIVED from angle+length — never stored, always in sync
    Vector2 end() {
        return { start.x + length * cos(angle),
                 start.y + length * sin(angle) };
    }

    void draw() {
        DrawLineV(start, end(), c);
        DrawCircleV(end(), r, c);
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
    // NOTE: this is a pure translation — y still points DOWN. +y = down the screen.
    Camera2D camera = { 0 };
    camera.offset = (Vector2){ GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f };
    camera.target = (Vector2){ 0.0f, 0.0f };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    SetTargetFPS(60);

    // UI
    bool dragging = false;

    // ---- Physics constants ----
    // g is in PIXELS/sec^2, not real 9.8 m/s^2 — our world is pixels, so we tune it
    // by feel. Bigger g = faster, snappier swing. Start here and adjust to taste.
    const float g = 800.0f;

    // Pendulum setup
    Pivot p1 (0, 0, 20, RED);

    float length = 150.0f;          // arm length in pixels
    float thetaInitial = 20.0f;     // degrees from horizontal — a near-horizontal start
                                    // so it has far to fall and swings dramatically

    Arm a1( {p1.x, p1.y},
            PI / 180.0f * thetaInitial,   // convert degrees -> radians for storage
            length,
            RAYWHITE );

    Arm a2( a1.end(),
            PI / 180.0f * thetaInitial,   // convert degrees -> radians for storage
            length,
            BLUE);

    // Main game loop
    while (!WindowShouldClose())
    {
        // ================= UPDATE =================

        // --- dt: seconds elapsed since last frame ---
        // Physics is written in "per second" units, but the loop runs per FRAME.
        // dt bridges them: multiplying by dt makes the motion frame-rate INDEPENDENT,
        // so the swing runs at the same real-world speed whether at 60 or 144 FPS.
        float dt = GetFrameTime();

        // --- Input: drag the pivot around (unchanged from Step 3) ---
        Vector2 world = GetScreenToWorld2D(GetMousePosition(), camera);

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (CheckCollisionPointCircle(world, { p1.x, p1.y }, p1.r)) {
                dragging = true;
            }
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            dragging = false;
        }
        if (dragging) {
            p1.x = world.x;
            p1.y = world.y;
            a1.start = { p1.x, p1.y };   // end() follows automatically
            a2.start = a1.end();
        }

        // --- PENDULUM PHYSICS ---
        // The whole swing is driven by ONE number changing over time: a1.angle.

        // 1. Convert our stored angle (from horizontal) into the angle-from-vertical
        //    that the pendulum equation expects. Straight-down = 90° from horizontal
        //    = 0 from vertical, so we subtract 90° (PI/2).
        float angleFromVertical = a1.angle - PI / 2.0f;

        // 2. Angular acceleration from the pendulum equation:
        //        α = -(g / L) * sin(θ_from_vertical)
        //    The sin() picks out gravity's TANGENTIAL component — the part
        //    perpendicular to the arm that actually torques it (the rod itself
        //    supplies the centripetal part, so we ignore that). The minus sign is
        //    the RESTORING force: it always pushes the angle back toward hanging down.
        //    At rest (straight down) sin(0)=0 → no acceleration → equilibrium.
        float angularAccel = -(g / a1.length) * sin(angleFromVertical);

        // 3. Integrate: acceleration changes velocity, velocity changes position.
        //    This is Euler integration — the simplest way to step a differential
        //    equation forward in time. Each frame nudges velocity by accel*dt,
        //    then nudges angle by velocity*dt.
        a1.angularVelocity += angularAccel * dt;
        a1.angle           += a1.angularVelocity * dt;

        //    NOTE: a real (frictionless) pendulum swings forever. If you'd like it
        //    to eventually settle, add a tiny damping factor that bleeds off energy:
        //        a1.angularVelocity *= 0.999f;
        //    Leave it out for now so you can clearly see the pure swing.

        // Same calculations for a2
        a2.start = a1.end();
        float angleFromVertical2 = a2.angle - PI / 2.0f;
        float angularAccel2 = -(g / a2.length) * sin(angleFromVertical2);
        a2.angularVelocity += angularAccel2 * dt;
        a2.angle           += a2.angularVelocity * dt;


        // ================= DRAW =================
        BeginDrawing();
            ClearBackground(DARKGRAY);
            BeginMode2D(camera);        // everything below is in centered coords

                p1.draw();
                a1.draw();
                a2.draw();

            EndMode2D();
        EndDrawing();
    }

    CloseWindow();
    return 0;
}