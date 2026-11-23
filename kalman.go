// Copyright 2026 The Accelerometer Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

// Process and measurement noise for one acceleration axis, in (m/s²)².
// At 60 Hz these settle to a gain of about 0.2, so jitter is reduced and a
// real change of acceleration is followed within a few samples.
const (
	accelProcessNoise     = 0.15 // variance added per second
	accelMeasurementNoise = 0.05 // variance of one sample

	// Gyro noise uses the same ratio, in (°/s)², so it settles to the same gain.
	gyroProcessNoise     = 3
	gyroMeasurementNoise = 1
)

// kalmanAxis estimates one acceleration axis. The state is the acceleration.
// Between samples it is a random walk, so the estimate can follow motion
// while rejecting measurement noise. q is process variance per second and r
// is the variance of one measurement.
type kalmanAxis struct {
	x     float64
	p     float64
	q     float64
	r     float64
	ready bool
}

func (a *kalmanAxis) reset() {
	*a = kalmanAxis{q: a.q, r: a.r}
}

func (a *kalmanAxis) update(z, dt float64) float64 {
	if !a.ready {
		a.x = z
		a.p = a.r
		a.ready = true
		return a.x
	}
	if dt < 0 {
		dt = 0
	}
	// Predict.
	a.p += a.q * dt
	// Update.
	k := a.p / (a.p + a.r)
	a.x += k * (z - a.x)
	a.p *= 1 - k
	return a.x
}

// kalman3 filters the three accelerometer axes independently.
type kalman3 struct {
	x, y, z kalmanAxis
}

func newKalman3(q, r float64) kalman3 {
	axis := func() kalmanAxis { return kalmanAxis{q: q, r: r} }
	return kalman3{x: axis(), y: axis(), z: axis()}
}

func (k *kalman3) reset() {
	k.x.reset()
	k.y.reset()
	k.z.reset()
}

func (k *kalman3) update(x, y, z, dt float64) (float64, float64, float64) {
	return k.x.update(x, dt), k.y.update(y, dt), k.z.update(z, dt)
}
