// Copyright 2026 The Accelerometer Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"math"
	"testing"
)

func TestQuatIntegrationTurnsXTowardY(t *testing.T) {
	q := quat{w: 1}
	dt := 0.01
	w := vec3{0, 0, math.Pi / 2}
	for i := 0; i < 100; i++ {
		q = q.integrate(w, dt)
	}
	got := q.rotate(vec3{1, 0, 0})
	if math.Abs(got.x) > 1e-6 || math.Abs(got.y-1) > 1e-6 || math.Abs(got.z) > 1e-6 {
		t.Fatalf("rotated x = %+v, want north", got)
	}
}

func TestLevelSpecificForceHasNoAcceleration(t *testing.T) {
	n := newINS()
	n.Predict(vec3{0, 0, gravity}, vec3{}, 0.02)
	n.Predict(vec3{0, 0, gravity}, vec3{}, 0.02)
	if math.Abs(n.v.x)+math.Abs(n.v.y)+math.Abs(n.v.z) > 1e-6 {
		t.Fatalf("velocity = %+v", n.v)
	}
}

func TestTiltedRestDoesNotRunAway(t *testing.T) {
	n := newINS()
	f := vec3{gravity, 0, 0}
	for i := 0; i < 200; i++ {
		n.Predict(f, vec3{}, 0.02)
	}
	if math.Hypot(n.p.x, n.p.y) > 0.5 || math.Abs(n.p.z) > 0.5 {
		t.Fatalf("position ran away: %+v", n.p)
	}
}

func TestINSIntegratesHorizontalAcceleration(t *testing.T) {
	n := newINS()
	n.Predict(vec3{0, 0, gravity}, vec3{}, 0.01)
	f := vec3{5, 0, gravity}
	const steps = 100
	const dt = 0.01
	for i := 0; i < steps; i++ {
		n.Predict(f, vec3{}, dt)
	}
	if math.Abs(n.p.x-2.5) > 1e-3 || math.Abs(n.v.x-5) > 1e-3 {
		t.Fatalf("p=%+v v=%+v, want east 2.5 m at 5 m/s", n.p, n.v)
	}
	if math.Abs(n.p.y) > 1e-3 || math.Abs(n.p.z) > 1e-2 {
		t.Fatalf("off-axis position %+v", n.p)
	}
}

func TestINSGPSMovesNorth(t *testing.T) {
	n := newINS()
	n.Predict(vec3{0, 0, gravity}, vec3{}, 0.02)
	n.UpdateGPS(37, -122, 10, true, 5)
	// A non-zero rate keeps the zero-velocity update from rejecting the move.
	n.Predict(vec3{0, 0, gravity}, vec3{0, 0, 0.5}, 0.02)
	const dLat = 11.1 / earthRadius * 180 / math.Pi
	for i := 0; i < 8; i++ {
		n.UpdateGPS(37+dLat, -122, 10, true, 5)
	}
	if n.p.y < 9 {
		t.Fatalf("north = %v, want about 11 m", n.p.y)
	}
}

func TestINSIgnoresGPSJitterWhileStill(t *testing.T) {
	n := newINS()
	for i := 0; i < 50; i++ {
		n.Predict(vec3{0, 0, gravity}, vec3{}, 0.02)
	}
	n.UpdateGPS(37, -122, 10, true, 5)
	const dLat = 3 / earthRadius * 180 / math.Pi
	for i := 0; i < 20; i++ {
		sign := 1.0
		if i%2 == 1 {
			sign = -1
		}
		n.Predict(vec3{0, 0, gravity}, vec3{}, 0.05)
		n.UpdateGPS(37+sign*dLat, -122, 10, true, 5)
	}
	if math.Hypot(n.p.x, n.p.y) > 1.5 {
		t.Fatalf("still position wandered to %+v", n.p)
	}
}

func TestInvert3(t *testing.T) {
	a := [3][3]float64{{4, 1, 0}, {1, 3, 1}, {0, 1, 2}}
	inv, ok := invert3(a, 3)
	if !ok {
		t.Fatal("not invertible")
	}
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			s := 0.0
			for k := 0; k < 3; k++ {
				s += inv[i][k] * a[k][j]
			}
			want := 0.0
			if i == j {
				want = 1
			}
			if math.Abs(s-want) > 1e-9 {
				t.Fatalf("inv*a [%d,%d] = %v", i, j, s)
			}
		}
	}
}
