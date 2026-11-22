// Copyright 2026 The Accelerometer Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"fmt"
	"syscall/js"
)

var (
	motionFunc js.Func
	listening  bool
	permWait   bool
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
	deviceMotionEvent := js.Global().Get("DeviceMotionEvent")
	if deviceMotionEvent.Type() != js.TypeFunction {
		fmt.Println("DeviceMotionEvent is not supported on this device.")
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
		}
		return nil
	})
	onError = js.FuncOf(func(this js.Value, args []js.Value) any {
		defer release()
		fmt.Println("Motion permission request failed:", jsDescribe(args))
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
	x, y, z, ok := readAcceleration(args[0])
	if !ok {
		return nil
	}
	text := fmt.Sprintf("X: %.2f, Y: %.2f, Z: %.2f", x, y, z)
	//js.Global().Get("alert").Invoke(fmt.Sprintf("X: %.2f, Y: %.2f, Z: %.2f\n", x, y, z))
	el := js.Global().Get("document").Call("getElementById", "measurements")
	if el.Type() == js.TypeObject {
		el.Set("textContent", text)
	}
	return nil
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
