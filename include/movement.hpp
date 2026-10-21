#pragma once

void initializeTransformations();
std::array<float, 3> calcMotorPercentages(int x, int y, int phoneHeading, int imuHeading);