// Copyright 2026 The Accelerometer Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"math"
	"testing"
)

func TestKalmanFirstSamplePassesThrough(t *testing.T) {
	f := newKalman3(accelProcessNoise, accelMeasurementNoise)
	x, y, z := f.update(1.25, -2.5, 9.81, 1.0/60)
	if x != 1.25 || y != -2.5 || z != 9.81 {
		t.Fatalf("first sample = %v %v %v", x, y, z)
	}
}

func TestKalmanSecondSampleIsPartialStep(t *testing.T) {
	dt := 1.0 / 60
	f := newKalman3(accelProcessNoise, accelMeasurementNoise)
	f.update(0, 0, 0, dt)
	x, y, z := f.update(10, -4, 2, dt)

	p := accelMeasurementNoise + accelProcessNoise*dt
	k := p / (p + accelMeasurementNoise)
	if math.Abs(x-10*k) > 1e-12 || math.Abs(y-(-4*k)) > 1e-12 || math.Abs(z-2*k) > 1e-12 {
		t.Fatalf("filtered = %v %v %v, gain %v", x, y, z, k)
	}
	if x <= 0 || x >= 10 {
		t.Fatalf("x = %v, want a value strictly between the old estimate and the sample", x)
	}
}

func TestKalmanTracksAConstant(t *testing.T) {
	f := newKalman3(accelProcessNoise, accelMeasurementNoise)
	var x float64
	for i := 0; i < 180; i++ {
		x, _, _ = f.update(9.81, 0, 0, 1.0/60)
	}
	if math.Abs(x-9.81) > 1e-6 {
		t.Fatalf("settled x = %v", x)
	}
}

func TestKalmanSmoothsNoise(t *testing.T) {
	f := newKalman3(accelProcessNoise, accelMeasurementNoise)
	const n = 200
	dt := 1.0 / 60
	var inMAD, outMAD float64
	for i := 0; i < n; i++ {
		z := 1.0
		if i%2 == 1 {
			z = -1
		}
		x, _, _ := f.update(z, 0, 0, dt)
		inMAD += math.Abs(z)
		outMAD += math.Abs(x)
	}
	inMAD /= n
	outMAD /= n
	if outMAD >= inMAD*0.5 {
		t.Fatalf("output MAD %v is not well below input MAD %v", outMAD, inMAD)
	}
}

func TestKalmanAxesAreIndependent(t *testing.T) {
	f := newKalman3(accelProcessNoise, accelMeasurementNoise)
	f.update(1, 2, 3, 1.0/60)
	x, y, z := f.update(5, 2, 3, 1.0/60)
	if y != 2 || z != 3 {
		t.Fatalf("y or z changed: %v %v", y, z)
	}
	if x == 1 || x == 5 {
		t.Fatalf("x = %v, want a filtered value", x)
	}
}

func TestKalmanLargerGapTrustsMeasurementMore(t *testing.T) {
	step := func(dt float64) float64 {
		f := newKalman3(accelProcessNoise, accelMeasurementNoise)
		f.update(0, 0, 0, dt)
		x, _, _ := f.update(10, 0, 0, dt)
		return x
	}
	if step(1.0/60) >= step(0.5) {
		t.Fatalf("short dt %v did not lag the long dt %v", step(1.0/60), step(0.5))
	}
}

func TestKalmanReset(t *testing.T) {
	f := newKalman3(accelProcessNoise, accelMeasurementNoise)
	f.update(4, 5, 6, 1.0/60)
	f.reset()
	x, y, z := f.update(1, 2, 3, 1.0/60)
	if x != 1 || y != 2 || z != 3 {
		t.Fatalf("after reset = %v %v %v", x, y, z)
	}
}
