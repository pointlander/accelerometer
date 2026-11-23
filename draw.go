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

var drawDebt float64

func noteDraw(dt float64) {
	drawDebt += dt
	if drawDebt < 0.1 {
		return
	}
	drawDebt = 0
	drawTrack()
}

func drawTrack() {
	canvas := js.Global().Get("document").Call("getElementById", "path")
	if canvas.Type() != js.TypeObject {
		return
	}
	ctx := canvas.Call("getContext", "2d")
	if ctx.Type() != js.TypeObject {
		return
	}
	w := canvas.Get("width").Float()
	h := canvas.Get("height").Float()
	if w < 2 || h < 2 {
		return
	}
	ctx.Set("fillStyle", "#f4f4f4")
	ctx.Call("fillRect", 0, 0, w, h)
	ctx.Set("fillStyle", "#222")
	ctx.Set("font", "14px sans-serif")
	ctx.Call("fillText", "N", w/2-4, 18)
	ctx.Call("fillText", "E", w-18, h/2+4)

	pts, cur, ok := nav.Track()
	if !ok {
		return
	}
	minE, maxE := cur.x, cur.x
	minN, maxN := cur.y, cur.y
	for _, p := range pts {
		minE = math.Min(minE, p.x)
		maxE = math.Max(maxE, p.x)
		minN = math.Min(minN, p.y)
		maxN = math.Max(maxN, p.y)
	}
	const minSpan = 20.0
	midE := 0.5 * (minE + maxE)
	midN := 0.5 * (minN + maxN)
	span := math.Max(maxE-minE, minSpan)
	span = math.Max(maxN-minN, span)
	minE, maxE = midE-span/2, midE+span/2
	minN, maxN = midN-span/2, midN+span/2

	const margin = 28.0
	plotW := w - 2*margin
	plotH := h - 2*margin
	mapX := func(e float64) float64 { return margin + (e-minE)/span*plotW }
	mapY := func(n float64) float64 { return h - margin - (n-minN)/span*plotH }

	ctx.Set("strokeStyle", "#e0e0e0")
	ctx.Set("lineWidth", 1)
	ctx.Call("beginPath")
	for i := 1; i < 4; i++ {
		x := margin + plotW*float64(i)/4
		y := margin + plotH*float64(i)/4
		ctx.Call("moveTo", x, margin)
		ctx.Call("lineTo", x, h-margin)
		ctx.Call("moveTo", margin, y)
		ctx.Call("lineTo", w-margin, y)
	}
	ctx.Call("stroke")

	ctx.Set("strokeStyle", "#1a73e8")
	ctx.Set("lineWidth", 2)
	ctx.Call("beginPath")
	started := false
	strokeTo := func(e, n float64) {
		x, y := mapX(e), mapY(n)
		if !started {
			ctx.Call("moveTo", x, y)
			started = true
			return
		}
		ctx.Call("lineTo", x, y)
	}
	for _, p := range pts {
		strokeTo(p.x, p.y)
	}
	if len(pts) == 0 || math.Hypot(cur.x-pts[len(pts)-1].x, cur.y-pts[len(pts)-1].y) > 0.05 {
		strokeTo(cur.x, cur.y)
	}
	ctx.Call("stroke")

	x, y := mapX(cur.x), mapY(cur.y)
	ctx.Set("fillStyle", "#d93025")
	ctx.Call("beginPath")
	ctx.Call("arc", x, y, 5, 0, 2*math.Pi)
	ctx.Call("fill")

	ctx.Set("fillStyle", "#222")
	ctx.Set("font", "12px sans-serif")
	ctx.Call("fillText", fmt.Sprintf("%.0f m", span), margin, h-8)
}
