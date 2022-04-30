//
// Created by wj on 22. 4. 30.
//

#include "../include/Boundary.h"

Boundary::Boundary()
	: mX(0.0f), mY(0.0f), mW(0.0f), mH(0.0f) {
}

Boundary::Boundary(float x, float z, float w, float h)
	: mX(x), mY(z), mW(w), mH(h) {
}

Boundary::Boundary(float x, float minY, float maxY)
	: mX(x / 2), mY((maxY + minY) / 2), mW(x / 2), mH((maxY - minY) / 2) {
}

float Boundary::GetX() const {
	return mX;
}

float Boundary::GetY() const {
	return mY;
}

float Boundary::GetW() const {
	return mW;
}

float Boundary::GetH() const {
	return mH;
}

void Boundary::SetBoundary(const float x, const float y, const float w, const float h) {
	mX = x;
	mY = y;
	mW = w;
	mH = h;
}

bool Boundary::IsConstained(Point3D *p) const {
	return (p->GetX() >= mX - mW && p->GetX() < mX + mW && p->GetY() >= mY - mH && p->GetY() < mY + mH);
}