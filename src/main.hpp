#pragma once

// Robot-side logic. The original project only computes the wheel matrix;
// it does not yet drive physical motors.
void initializeDrive();
void updateDrive();
void motor_matrix_translation(int x, int y, int heading);
