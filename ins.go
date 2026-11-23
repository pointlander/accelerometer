// Copyright 2026 The Accelerometer Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import "math"

const (
	gravity     = 9.80665
	earthRadius = 6378137.0
)

// vec3 is a right-handed vector. Body axes match the device: x right, y
// toward the top of the screen, z out of the screen. Navigation axes are
// east, north, and up.
type vec3 struct{ x, y, z float64 }

func (a vec3) add(b vec3) vec3      { return vec3{a.x + b.x, a.y + b.y, a.z + b.z} }
func (a vec3) sub(b vec3) vec3      { return vec3{a.x - b.x, a.y - b.y, a.z - b.z} }
func (a vec3) scale(s float64) vec3 { return vec3{a.x * s, a.y * s, a.z * s} }
func (a vec3) dot(b vec3) float64   { return a.x*b.x + a.y*b.y + a.z*b.z }
func (a vec3) cross(b vec3) vec3 {
	return vec3{a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x}
}
func (a vec3) norm() float64 { return math.Hypot(a.x, math.Hypot(a.y, a.z)) }

type vec2 struct{ x, y float64 }

// quat rotates body vectors into the navigation frame. w is the scalar part.
type quat struct{ w, x, y, z float64 }

func (q quat) conj() quat { return quat{q.w, -q.x, -q.y, -q.z} }

func (q quat) normalize() quat {
	n := math.Hypot(math.Hypot(q.w, q.x), math.Hypot(q.y, q.z))
	if n == 0 {
		return quat{w: 1}
	}
	s := 1 / n
	return quat{q.w * s, q.x * s, q.y * s, q.z * s}
}

func (q quat) mul(p quat) quat {
	return quat{
		w: q.w*p.w - q.x*p.x - q.y*p.y - q.z*p.z,
		x: q.w*p.x + q.x*p.w + q.y*p.z - q.z*p.y,
		y: q.w*p.y - q.x*p.z + q.y*p.w + q.z*p.x,
		z: q.w*p.z + q.x*p.y - q.y*p.x + q.z*p.w,
	}
}

func (q quat) rotate(v vec3) vec3 {
	r := q.mul(quat{0, v.x, v.y, v.z}).mul(q.conj())
	return vec3{r.x, r.y, r.z}
}

func (q quat) rotateInv(v vec3) vec3 { return q.conj().rotate(v) }

// integrate applies a body-frame angular rate (rad/s) over dt.
func (q quat) integrate(w vec3, dt float64) quat {
	angle := w.norm() * dt
	var dq quat
	if angle < 1e-8 {
		dq = quat{1, w.x * dt / 2, w.y * dt / 2, w.z * dt / 2}
	} else {
		s := math.Sin(angle/2) / (angle / dt)
		dq = quat{math.Cos(angle / 2), w.x * s, w.y * s, w.z * s}
	}
	return q.mul(dq).normalize()
}

// quatFromEuler builds a body-to-nav rotation. roll, pitch, and yaw are
// radians about body x, y, and z. The matrix is Rz(yaw) Ry(pitch) Rx(roll).
func quatFromEuler(roll, pitch, yaw float64) quat {
	cr, sr := math.Cos(roll/2), math.Sin(roll/2)
	cp, sp := math.Cos(pitch/2), math.Sin(pitch/2)
	cy, sy := math.Cos(yaw/2), math.Sin(yaw/2)
	return quat{
		w: cr*cp*cy + sr*sp*sy,
		x: sr*cp*cy - cr*sp*sy,
		y: cr*sp*cy + sr*cp*sy,
		z: cr*cp*sy - sr*sp*cy,
	}.normalize()
}

// attitudeFromAccel levels the device so the measured specific force points up.
// Yaw stays zero because gravity does not observe it.
func attitudeFromAccel(f vec3) quat {
	n := f.norm()
	if n < 1 {
		return quat{w: 1}
	}
	roll := math.Atan2(f.y, f.z)
	pitch := math.Atan2(-f.x, math.Hypot(f.y, f.z))
	return quatFromEuler(roll, pitch, 0)
}

// INS is a loosely coupled inertial navigator. Gyro and accelerometer
// propagate attitude, velocity, and position. A Kalman filter corrects
// position from GPS and velocity when the device is still.
type INS struct {
	ready     bool
	q         quat
	p         vec3
	v         vec3
	bias      vec3
	P         [6][6]float64
	haveCov   bool
	stillTime float64

	hasOrigin bool
	lat0      float64
	lon0      float64
	alt0      float64

	track    []vec2
	last     vec2
	haveLast bool
}

func newINS() INS {
	n := INS{q: quat{w: 1}}
	n.initP(100)
	return n
}

func (n *INS) initP(posStd float64) {
	n.P = [6][6]float64{}
	s2 := posStd * posStd
	n.P[0][0], n.P[1][1], n.P[2][2] = s2, s2, s2
	n.P[3][3], n.P[4][4], n.P[5][5] = 1, 1, 1
	n.haveCov = true
}

// Predict integrates one IMU sample. f is specific force including gravity,
// in m/s². w is the angular rate around the body x, y, and z axes, in rad/s.
func (n *INS) Predict(f, w vec3, dt float64) {
	if dt <= 0 {
		return
	}
	if dt > 0.25 {
		dt = 0.25
	}
	if !n.ready {
		if f.norm() < 1 {
			return
		}
		n.q = attitudeFromAccel(f)
		n.ready = true
		if !n.haveCov {
			n.initP(100)
		}
		n.samplePath()
		return
	}

	corrected := w.sub(n.bias)
	stationary := math.Abs(f.norm()-gravity) < 0.35 && corrected.norm() < 0.08
	if stationary {
		n.stillTime += dt
	} else {
		n.stillTime = 0
	}
	if n.stillTime > 0.5 {
		a := dt / 2
		if a > 0.05 {
			a = 0.05
		}
		n.bias = n.bias.add(w.sub(n.bias).scale(a))
		corrected = w.sub(n.bias)
	}

	n.q = n.q.integrate(corrected, dt)
	if stationary {
		n.correctTilt(f, 0.08)
	}

	// a = R f + g, with g pointing down. At rest, level, f is +gravity on z.
	acc := n.q.rotate(f).add(vec3{0, 0, -gravity})
	if n.stillTime > 0.4 {
		acc = vec3{}
	}
	n.p = n.p.add(n.v.scale(dt)).add(acc.scale(0.5 * dt * dt))
	n.v = n.v.add(acc.scale(dt))
	n.predictCov(dt)
	if n.stillTime > 0.4 {
		n.update([]int{3, 4, 5}, []float64{-n.v.x, -n.v.y, -n.v.z}, 0.05*0.05)
	}
	n.samplePath()
}

// UpdateGPS applies a WGS84 fix. lat and lon are degrees. alt is meters and
// is used only when hasAlt is set. accuracy is the horizontal standard
// deviation in meters.
func (n *INS) UpdateGPS(lat, lon, alt float64, hasAlt bool, accuracy float64) {
	if accuracy <= 0 {
		accuracy = 10
	}
	if !n.hasOrigin {
		n.lat0, n.lon0, n.alt0 = lat, lon, alt
		if !hasAlt {
			n.alt0 = 0
		}
		n.hasOrigin = true
		n.p = vec3{}
		n.initP(accuracy)
		n.track = nil
		n.haveLast = false
		n.samplePath()
		return
	}
	meas := n.enu(lat, lon, alt)
	innovE := meas.x - n.p.x
	innovN := meas.y - n.p.y
	horiz := math.Hypot(innovE, innovN)
	// A resting phone's GPS wanders by a few meters. Ignore that, but accept
	// a real relocation.
	if n.stillTime > 0.4 && horiz < 4 {
		return
	}
	if hasAlt {
		n.update([]int{0, 1, 2}, []float64{innovE, innovN, meas.z - n.p.z}, accuracy*accuracy)
	} else {
		n.update([]int{0, 1}, []float64{innovE, innovN}, accuracy*accuracy)
	}
	n.samplePath()
}

func (n *INS) enu(lat, lon, alt float64) vec3 {
	dLat := (lat - n.lat0) * math.Pi / 180
	dLon := (lon - n.lon0) * math.Pi / 180
	lat0 := n.lat0 * math.Pi / 180
	north := dLat * earthRadius
	east := dLon * earthRadius * math.Cos(lat0)
	up := alt - n.alt0
	return vec3{east, north, up}
}

func (n *INS) correctTilt(f vec3, gain float64) {
	mag := f.norm()
	if mag < 1 {
		return
	}
	pred := n.q.rotateInv(vec3{0, 0, 1})
	meas := f.scale(1 / mag)
	axis := pred.cross(meas)
	corr := quat{1, axis.x * gain / 2, axis.y * gain / 2, axis.z * gain / 2}.normalize()
	n.q = n.q.mul(corr).normalize()
}

func (n *INS) predictCov(dt float64) {
	var F [6][6]float64
	for i := 0; i < 6; i++ {
		F[i][i] = 1
	}
	F[0][3], F[1][4], F[2][5] = dt, dt, dt
	FP := mul6(F, n.P)
	n.P = mul6(FP, transpose6(F))
	sa2 := 0.5 * 0.5
	n.P[3][3] += sa2 * dt * dt
	n.P[4][4] += sa2 * dt * dt
	n.P[5][5] += sa2 * dt * dt
	step := 0.25 * sa2 * dt * dt * dt * dt
	n.P[0][0] += step
	n.P[1][1] += step
	n.P[2][2] += step
}

// update applies a measurement of the selected state entries. innov is
// measured minus predicted. variance is the variance of each component.
func (n *INS) update(indices []int, innov []float64, variance float64) {
	m := len(indices)
	if m == 0 || m > 3 || variance <= 0 {
		return
	}
	var S [3][3]float64
	for i := 0; i < m; i++ {
		for j := 0; j < m; j++ {
			S[i][j] = n.P[indices[i]][indices[j]]
		}
		S[i][i] += variance
	}
	Sinv, ok := invert3(S, m)
	if !ok {
		return
	}
	var K [6][3]float64
	for r := 0; r < 6; r++ {
		for c := 0; c < m; c++ {
			sum := 0.0
			for k := 0; k < m; k++ {
				sum += n.P[r][indices[k]] * Sinv[k][c]
			}
			K[r][c] = sum
		}
	}
	var dx [6]float64
	for r := 0; r < 6; r++ {
		for c := 0; c < m; c++ {
			dx[r] += K[r][c] * innov[c]
		}
	}
	n.p = n.p.add(vec3{dx[0], dx[1], dx[2]})
	n.v = n.v.add(vec3{dx[3], dx[4], dx[5]})

	var IKH [6][6]float64
	for i := 0; i < 6; i++ {
		IKH[i][i] = 1
	}
	for r := 0; r < 6; r++ {
		for c := 0; c < m; c++ {
			IKH[r][indices[c]] -= K[r][c]
		}
	}
	P := mul6(mul6(IKH, n.P), transpose6(IKH))
	for i := 0; i < 6; i++ {
		for j := 0; j < 6; j++ {
			s := 0.0
			for c := 0; c < m; c++ {
				s += K[i][c] * variance * K[j][c]
			}
			P[i][j] += s
		}
	}
	for i := 0; i < 6; i++ {
		for j := i + 1; j < 6; j++ {
			s := 0.5 * (P[i][j] + P[j][i])
			P[i][j], P[j][i] = s, s
		}
	}
	n.P = P
}

func (n *INS) samplePath() {
	cur := vec2{n.p.x, n.p.y}
	if n.haveLast && math.Hypot(cur.x-n.last.x, cur.y-n.last.y) < 0.25 {
		return
	}
	n.track = append(n.track, cur)
	if len(n.track) > 4000 {
		n.track = append([]vec2(nil), n.track[len(n.track)-4000:]...)
	}
	n.last = cur
	n.haveLast = true
}

// Track returns the east/north samples and the current horizontal position.
func (n *INS) Track() (pts []vec2, cur vec2, ok bool) {
	if !n.ready && !n.hasOrigin {
		return nil, vec2{}, false
	}
	return n.track, vec2{n.p.x, n.p.y}, true
}

func mul6(a, b [6][6]float64) [6][6]float64 {
	var c [6][6]float64
	for i := 0; i < 6; i++ {
		for k := 0; k < 6; k++ {
			if a[i][k] == 0 {
				continue
			}
			for j := 0; j < 6; j++ {
				c[i][j] += a[i][k] * b[k][j]
			}
		}
	}
	return c
}

func transpose6(a [6][6]float64) [6][6]float64 {
	var b [6][6]float64
	for i := 0; i < 6; i++ {
		for j := 0; j < 6; j++ {
			b[i][j] = a[j][i]
		}
	}
	return b
}

// invert3 inverts the leading m×m block of a, m is 2 or 3.
func invert3(a [3][3]float64, m int) ([3][3]float64, bool) {
	var inv [3][3]float64
	switch m {
	case 2:
		det := a[0][0]*a[1][1] - a[0][1]*a[1][0]
		if math.Abs(det) < 1e-12 {
			return inv, false
		}
		s := 1 / det
		inv[0][0] = a[1][1] * s
		inv[0][1] = -a[0][1] * s
		inv[1][0] = -a[1][0] * s
		inv[1][1] = a[0][0] * s
		return inv, true
	case 3:
		det := a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1]) -
			a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0]) +
			a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0])
		if math.Abs(det) < 1e-12 {
			return inv, false
		}
		s := 1 / det
		inv[0][0] = (a[1][1]*a[2][2] - a[1][2]*a[2][1]) * s
		inv[0][1] = (a[0][2]*a[2][1] - a[0][1]*a[2][2]) * s
		inv[0][2] = (a[0][1]*a[1][2] - a[0][2]*a[1][1]) * s
		inv[1][0] = (a[1][2]*a[2][0] - a[1][0]*a[2][2]) * s
		inv[1][1] = (a[0][0]*a[2][2] - a[0][2]*a[2][0]) * s
		inv[1][2] = (a[0][2]*a[1][0] - a[0][0]*a[1][2]) * s
		inv[2][0] = (a[1][0]*a[2][1] - a[1][1]*a[2][0]) * s
		inv[2][1] = (a[0][1]*a[2][0] - a[0][0]*a[2][1]) * s
		inv[2][2] = (a[0][0]*a[1][1] - a[0][1]*a[1][0]) * s
		return inv, true
	default:
		return inv, false
	}
}
