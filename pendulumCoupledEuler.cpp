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
        DrawCircleV(end(), r, c);
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

    // ---- Physics constants ----
    const float g = 800.0f;   // pixels/sec^2 — tuned by feel, not real 9.8

    // Pendulum setup
    Pivot p1 (0, 0, 10, RED);

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

    // Main game loop
    while (!WindowShouldClose())
    {
        // ================= UPDATE =================

        float dt = GetFrameTime();

        // --- Input: drag the pivot around ---
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
            a1.start = { p1.x, p1.y };
            a2.start = a1.end();
        }

        // --- PENDULUM PHYSICS ---
        float angleFromVertical  = a1.angle - PI / 2.0f;
        float angleFromVertical2 = a2.angle - PI / 2.0f;
        float deltaAngle = angleFromVertical - angleFromVertical2;

        float num1 = -g * (2.0f * a1.mass + a2.mass) * sin(angleFromVertical)
                    - a2.mass * g * sin(angleFromVertical - 2.0f * angleFromVertical2)
                    - 2.0f * sin(deltaAngle) * a2.mass * (a2.angularVelocity * a2.angularVelocity * a2.length + a1.angularVelocity * a1.angularVelocity * a1.length * cos(deltaAngle));

        float den1 = a1.length * (2.0f * a1.mass + a2.mass - a2.mass * cos(2.0f * angleFromVertical - 2.0f * angleFromVertical2));

        float angularAccel = num1 / den1;

        float num2 = 2.0f * sin(deltaAngle) * (a1.angularVelocity * a1.angularVelocity * a1.length * (a1.mass + a2.mass)
             + g * (a1.mass + a2.mass) * cos(angleFromVertical)
             + a2.angularVelocity * a2.angularVelocity * a2.length * a2.mass * cos(deltaAngle));

        float den2 = a2.length * (2.0f * a1.mass + a2.mass - a2.mass * cos(2.0f * angleFromVertical - 2.0f * angleFromVertical2));

        float angularAccel2 = num2 / den2;

        // Integrate (Euler-Cromer: new velocity, then position)
        a1.angularVelocity += angularAccel * dt;
        a1.angle           += a1.angularVelocity * dt;

        a2.start = a1.end();

        a2.angularVelocity += angularAccel2 * dt;
        a2.angle           += a2.angularVelocity * dt;

        // --- Energy calculations (scaled by /10000 to keep magnitudes readable) ---
        float kineticEnergy = (0.5f * (a1.mass + a2.mass) * a1.length * a1.length * a1.angularVelocity * a1.angularVelocity
                             + 0.5f * a2.mass * a2.length * a2.length * a2.angularVelocity * a2.angularVelocity
                             + a2.mass * a1.length * a2.length * a1.angularVelocity * a2.angularVelocity * cos(deltaAngle)) / 10000.0f;

        float potentialEnergy = (-g * (a1.mass + a2.mass) * a1.length * cos(angleFromVertical)
                               - g * a2.mass * a2.length * cos(angleFromVertical2)) / 10000.0f;

        float totalEnergy = kineticEnergy + potentialEnergy;

        // Capture the TRUE launch energy exactly once, then never touch it again.
        // This fixed baseline is what makes drift actually expose energy bleed.
        if (firstFrame) {
            initialEnergy = totalEnergy;
            firstFrame = false;
        }

        // NOTE: no longer accumulating an energy history.
        // energy.push_back(totalEnergy);

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