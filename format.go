// Copyright 2026 The Accelerometer Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import "fmt"

// formatFloat prints v with a sign column, three digits, and three decimals.
// Positive numbers use a leading space, so the result is the same length as
// the negative form. Values with more than three digits before the decimal
// grow on the left and stay the same length as their negative counterpart.
func formatFloat(v float64) string {
	sign := " "
	if v < 0 {
		sign = "-"
		v = -v
	}
	return sign + fmt.Sprintf("%07.3f", v)
}
