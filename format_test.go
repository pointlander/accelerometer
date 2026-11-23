// Copyright 2026 The Accelerometer Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import "testing"

func TestFormatFloat(t *testing.T) {
	tests := []struct {
		in   float64
		want string
	}{
		{0, " 000.000"},
		{1.25, " 001.250"},
		{-2.5, "-002.500"},
		{9.81, " 009.810"},
		{37.8199, " 037.820"},
		{-122.4783, "-122.478"},
		{1000, " 1000.000"},
		{-1000, "-1000.000"},
	}
	for _, tt := range tests {
		if got := formatFloat(tt.in); got != tt.want {
			t.Fatalf("formatFloat(%v) = %q, want %q", tt.in, got, tt.want)
		}
	}
	if len(formatFloat(1.25)) != len(formatFloat(-2.5)) {
		t.Fatalf("positive and negative lengths differ: %q %q", formatFloat(1.25), formatFloat(-2.5))
	}
	if len(formatFloat(1000)) != len(formatFloat(-1000)) {
		t.Fatalf("wide positive and negative lengths differ: %q %q", formatFloat(1000), formatFloat(-1000))
	}
}
