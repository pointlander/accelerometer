// Copyright 2026 The Accelerometer Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"net/http"
	"net/http/httptest"
	"path/filepath"
	"reflect"
	"testing"
)

func TestParseHosts(t *testing.T) {
	got := parseHosts(" example.com, www.example.com:443, ,")
	want := []string{"example.com", "www.example.com"}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("parseHosts = %q, want %q", got, want)
	}
	if hosts := parseHosts("  "); len(hosts) != 0 {
		t.Fatalf("blank host list = %q", hosts)
	}
}

func TestRedirectToHTTPS(t *testing.T) {
	tests := []struct {
		addr string
		host string
		want string
	}{
		{addr: ":443", host: "example.com", want: "https://example.com/index.html?x=1"},
		{addr: ":443", host: "example.com:80", want: "https://example.com/index.html?x=1"},
		{addr: "127.0.0.1:8443", host: "example.com:8080", want: "https://example.com:8443/index.html?x=1"},
	}
	for _, tt := range tests {
		req := httptest.NewRequest(http.MethodGet, "/index.html?x=1", nil)
		req.Host = tt.host
		rec := httptest.NewRecorder()
		redirectToHTTPS(tt.addr).ServeHTTP(rec, req)
		if rec.Code != http.StatusMovedPermanently {
			t.Fatalf("%s %s: status %d", tt.addr, tt.host, rec.Code)
		}
		if got := rec.Header().Get("Location"); got != tt.want {
			t.Fatalf("%s %s: location %q, want %q", tt.addr, tt.host, got, tt.want)
		}
	}
}

func TestWebFiles(t *testing.T) {
	srv := httptest.NewServer(http.FileServer(http.Dir(filepath.Join("..", "web"))))
	t.Cleanup(srv.Close)

	res, err := http.Get(srv.URL + "/")
	if err != nil {
		t.Fatal(err)
	}
	res.Body.Close()
	if res.StatusCode != http.StatusOK {
		t.Fatalf("index status %d", res.StatusCode)
	}
	if ct := res.Header.Get("Content-Type"); ct != "text/html; charset=utf-8" {
		t.Fatalf("index content type %q", ct)
	}

	res, err = http.Get(srv.URL + "/main.wasm")
	if err != nil {
		t.Fatal(err)
	}
	res.Body.Close()
	if res.StatusCode != http.StatusOK {
		t.Fatalf("wasm status %d", res.StatusCode)
	}
	if ct := res.Header.Get("Content-Type"); ct != "application/wasm" {
		t.Fatalf("wasm content type %q", ct)
	}
}
