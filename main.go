// Copyright 2026 The Accelerometer Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

//go:build js && wasm

package main

import (
	"fmt"
	"math"
	"syscall/js"
)

var (
	motionFunc js.Func
	listening  bool
	permWait   bool

	measurementFilter = newKalman3(accelProcessNoise, accelMeasurementNoise)
	gyroFilter        = newKalman3(gyroProcessNoise, gyroMeasurementNoise)

	gpsSuccess  js.Func
	gpsFailure  js.Func
	gpsWatching bool

	accelLine string
	gyroLine  string
	gpsLine   string
	navLine   string

	nav = newINS()
)

func main() {
	// Expose the initialization function to the browser's global window object.
	js.Global().Set("startMotionListener", js.FuncOf(startMotionListener))

	// Keep the Go WASM runtime alive.
	select {}
}

func startMotionListener(this js.Value, args []js.Value) any {
	if listening || permWait {
		return nil
	}
	//js.Global().Get("alert").Invoke("1")
	if !gpsWatching {
		nav = newINS()
		navLine = ""
		drawDebt = 0
		watchGPS()
	}
	deviceMotionEvent := js.Global().Get("DeviceMotionEvent")
	if deviceMotionEvent.Type() != js.TypeFunction {
		fmt.Println("DeviceMotionEvent is not supported on this device.")
		accelLine = "Accel: not supported"
		gyroLine = "Gyro: not supported"
		renderMeasurements()
		return nil
	}
	//js.Global().Get("alert").Invoke("2")
	// iOS requires requestPermission, and it must run inside this user gesture.
	// Other browsers do not implement it and deliver events without a prompt.
	reqPerm := deviceMotionEvent.Get("requestPermission")
	if reqPerm.Type() != js.TypeFunction {
		attachListener()
		return nil
	}
	//js.Global().Get("alert").Invoke("3")
	permWait = true
	promise, err := callJS(deviceMotionEvent, "requestPermission")
	if err != nil {
		permWait = false
		fmt.Println("Motion permission request failed:", err)
		markMotionUnavailable("permission failed")
		return nil
	}
	//js.Global().Get("alert").Invoke("4")
	var onResult, onError js.Func
	released := false
	release := func() {
		if released {
			return
		}
		released = true
		permWait = false
		onResult.Release()
		onError.Release()
	}
	onResult = js.FuncOf(func(this js.Value, args []js.Value) any {
		defer release()
		state := ""
		if len(args) > 0 && args[0].Type() == js.TypeString {
			state = args[0].String()
		}
		if state == "granted" {
			attachListener()
		} else {
			fmt.Println("Motion permission denied.")
			markMotionUnavailable("permission denied")
		}
		return nil
	})
	onError = js.FuncOf(func(this js.Value, args []js.Value) any {
		defer release()
		fmt.Println("Motion permission request failed:", jsDescribe(args))
		markMotionUnavailable("permission failed")
		return nil
	})
	//js.Global().Get("alert").Invoke("5")
	next, err := callJS(promise, "then", onResult)
	if err != nil {
		release()
		fmt.Println("Motion permission request failed:", err)
		return nil
	}
	if _, err := callJS(next, "catch", onError); err != nil {
		// then is already registered. Drop the unused rejection handler.
		onError.Release()
		fmt.Println("Motion permission request failed:", err)
	}
	return nil
}

func attachListener() {
	if listening {
		return
	}
	measurementFilter.reset()
	gyroFilter.reset()
	motionFunc = js.FuncOf(onMotion)
	if _, err := callJS(js.Global(), "addEventListener", "devicemotion", motionFunc); err != nil {
		motionFunc.Release()
		fmt.Println("Could not listen for device motion:", err)
		return
	}
	listening = true
	//js.Global().Get("alert").Invoke("6")
}

func onMotion(this js.Value, args []js.Value) any {
	if len(args) == 0 || args[0].Type() != js.TypeObject {
		return nil
	}
	event := args[0]
	dt := motionInterval(event)
	ax, ay, az, accelOK := readAcceleration(event)
	alpha, beta, gamma, gyroOK := readRotationRate(event)
	if accelOK {
		x, y, z := measurementFilter.update(ax, ay, az, dt)
		accelLine = fmt.Sprintf("Accel: X: %s, Y: %s, Z: %s m/s²", formatFloat(x), formatFloat(y), formatFloat(z))
		//js.Global().Get("alert").Invoke(fmt.Sprintf("X: %.2f, Y: %.2f, Z: %.2f\n", x, y, z))
	}
	if gyroOK {
		ga, gb, gg := gyroFilter.update(alpha, beta, gamma, dt)
		gyroLine = fmt.Sprintf("Gyro: alpha: %s, beta: %s, gamma: %s °/s", formatFloat(ga), formatFloat(gb), formatFloat(gg))
	} else if gyroLine == "" {
		gyroLine = "Gyro: unavailable"
	}
	if accelOK {
		const deg = math.Pi / 180
		w := vec3{}
		if gyroOK {
			// rotationRate is degrees per second about z, x, y.
			w = vec3{beta * deg, gamma * deg, alpha * deg}
		}
		wasReady := nav.ready
		nav.Predict(vec3{ax, ay, az}, w, dt)
		updateNavLine()
		if !wasReady {
			drawTrack()
		} else {
			noteDraw(dt)
		}
	}
	renderMeasurements()
	return nil
}

func markMotionUnavailable(reason string) {
	if accelLine == "" {
		accelLine = "Accel: " + reason
	}
	if gyroLine == "" {
		gyroLine = "Gyro: " + reason
	}
	renderMeasurements()
}

func watchGPS() {
	if gpsWatching {
		return
	}
	gpsWatching = true
	geo := js.Global().Get("navigator").Get("geolocation")
	if geo.Type() != js.TypeObject {
		gpsLine = "GPS: not supported"
		renderMeasurements()
		return
	}
	gpsSuccess = js.FuncOf(func(this js.Value, args []js.Value) any {
		if len(args) == 0 || args[0].Type() != js.TypeObject {
			return nil
		}
		coords := args[0].Get("coords")
		if coords.Type() != js.TypeObject {
			gpsLine = "GPS: unavailable"
		} else {
			gpsLine = formatGPS(coords)
			if lat, latOK := jsFloat(coords.Get("latitude")); latOK {
				if lon, lonOK := jsFloat(coords.Get("longitude")); lonOK {
					alt, altOK := jsFloat(coords.Get("altitude"))
					accuracy := 10.0
					if a, ok := jsFloat(coords.Get("accuracy")); ok && a > 0 {
						accuracy = a
					}
					nav.UpdateGPS(lat, lon, alt, altOK, accuracy)
					updateNavLine()
					drawTrack()
				}
			}
		}
		renderMeasurements()
		return nil
	})
	gpsFailure = js.FuncOf(func(this js.Value, args []js.Value) any {
		gpsLine = "GPS: " + jsDescribe(args)
		renderMeasurements()
		return nil
	})
	_, err := callJS(geo, "watchPosition", gpsSuccess, gpsFailure, map[string]any{
		"enableHighAccuracy": true,
		"maximumAge":         0,
		"timeout":            20000,
	})
	if err != nil {
		gpsSuccess.Release()
		gpsFailure.Release()
		gpsWatching = false
		gpsLine = "GPS: " + err.Error()
		renderMeasurements()
	}
}

func formatGPS(coords js.Value) string {
	lat, latOK := jsFloat(coords.Get("latitude"))
	lon, lonOK := jsFloat(coords.Get("longitude"))
	if !latOK || !lonOK {
		return "GPS: unavailable"
	}
	text := fmt.Sprintf("GPS: %s, %s", formatFloat(lat), formatFloat(lon))
	if accuracy, ok := jsFloat(coords.Get("accuracy")); ok {
		text += fmt.Sprintf(" (±%s m)", formatFloat(accuracy))
	}
	if altitude, ok := jsFloat(coords.Get("altitude")); ok {
		text += fmt.Sprintf(", alt %s m", formatFloat(altitude))
	}
	return text
}

func renderMeasurements() {
	accel := accelLine
	if accel == "" {
		accel = "Accel: waiting"
	}
	gyro := gyroLine
	if gyro == "" {
		gyro = "Gyro: waiting"
	}
	gps := gpsLine
	if gps == "" {
		gps = "GPS: waiting"
	}
	navText := navLine
	if navText == "" {
		navText = "Nav: waiting"
	}
	el := js.Global().Get("document").Call("getElementById", "measurements")
	if el.Type() == js.TypeObject {
		el.Set("textContent", accel+"\n"+gyro+"\n"+gps+"\n"+navText)
	}
}

func updateNavLine() {
	if !nav.ready && !nav.hasOrigin {
		return
	}
	navLine = fmt.Sprintf("Nav: E:%s, N:%s, U:%s m", formatFloat(nav.p.x), formatFloat(nav.p.y), formatFloat(nav.p.z))
}

// readAcceleration prefers accelerationIncludingGravity and falls back to
// acceleration. Either object, and any axis, may be null.
func readAcceleration(event js.Value) (x, y, z float64, ok bool) {
	for _, name := range []string{"accelerationIncludingGravity", "acceleration"} {
		accel := event.Get(name)
		if accel.Type() != js.TypeObject {
			continue
		}
		xv, xok := jsFloat(accel.Get("x"))
		yv, yok := jsFloat(accel.Get("y"))
		zv, zok := jsFloat(accel.Get("z"))
		if xok && yok && zok {
			return xv, yv, zv, true
		}
	}
	return 0, 0, 0, false
}

// readRotationRate reads the gyroscope angular velocity in degrees per second.
// Any axis may be null.
func readRotationRate(event js.Value) (alpha, beta, gamma float64, ok bool) {
	rate := event.Get("rotationRate")
	if rate.Type() != js.TypeObject {
		return 0, 0, 0, false
	}
	alpha, aok := jsFloat(rate.Get("alpha"))
	beta, bok := jsFloat(rate.Get("beta"))
	gamma, gok := jsFloat(rate.Get("gamma"))
	if !aok || !bok || !gok {
		return 0, 0, 0, false
	}
	return alpha, beta, gamma, true
}

// motionInterval returns the sample period in seconds. DeviceMotionEvent
// reports interval in milliseconds, and some browsers report 0.
func motionInterval(event js.Value) float64 {
	const fallback = 1.0 / 60
	ms, ok := jsFloat(event.Get("interval"))
	if !ok || ms <= 0 || ms > 1000 {
		return fallback
	}
	return ms / 1000
}

func jsFloat(v js.Value) (float64, bool) {
	if v.Type() != js.TypeNumber {
		return 0, false
	}
	return v.Float(), true
}

// callJS invokes a JavaScript method and converts a thrown exception into an error.
// syscall/js panics on exceptions, which would exit the WebAssembly program.
func callJS(recv js.Value, method string, args ...any) (js.Value, error) {
	var result js.Value
	var caught error
	func() {
		defer func() {
			if r := recover(); r != nil {
				caught = jsRecover(r)
			}
		}()
		result = recv.Call(method, args...)
	}()
	return result, caught
}

func jsRecover(r any) error {
	if e, ok := r.(js.Error); ok {
		msg := e.Get("message")
		if msg.Type() == js.TypeString {
			return fmt.Errorf("JavaScript error: %s", msg.String())
		}
		return fmt.Errorf("JavaScript error")
	}
	if err, ok := r.(error); ok {
		return err
	}
	return fmt.Errorf("%v", r)
}

func jsDescribe(args []js.Value) string {
	if len(args) == 0 {
		return "unknown error"
	}
	v := args[0]
	switch v.Type() {
	case js.TypeString:
		return v.String()
	case js.TypeObject:
		msg := v.Get("message")
		if msg.Type() == js.TypeString && msg.String() != "" {
			return msg.String()
		}
		text, err := callJS(v, "toString")
		if err == nil && text.Type() == js.TypeString {
			return text.String()
		}
	}
	return "unknown error"
}
