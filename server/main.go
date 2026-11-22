// Copyright 2026 The Accelerometer Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Command server serves the web directory over HTTPS.
// Certificates are obtained and renewed with Let's Encrypt via autocert.
//
// Ports 80 and 443 must be reachable from the internet for the domain
// passed to -host. Port 80 answers ACME HTTP-01 challenges and redirects
// other requests to HTTPS.
package main

import (
	"crypto/tls"
	"flag"
	"fmt"
	"log"
	"net"
	"net/http"
	"net/url"
	"os"
	"path/filepath"
	"strings"
	"time"

	"golang.org/x/crypto/acme/autocert"
)

func main() {
	hostFlag := flag.String("host", "", "comma-separated domain names to obtain certificates for")
	email := flag.String("email", "", "contact email for Let's Encrypt (optional)")
	webDir := flag.String("web", "web", "directory to serve")
	cacheDir := flag.String("cache", "certs", "directory for cached certificates")
	httpsAddr := flag.String("https", ":443", "HTTPS listen address")
	httpAddr := flag.String("http", ":80", "HTTP listen address for ACME challenges and redirects")
	flag.Usage = func() {
		fmt.Fprintf(flag.CommandLine.Output(), "Usage: server -host example.com [flags]\n\nServes the web directory over HTTPS using a Let's Encrypt certificate.\n\n")
		flag.PrintDefaults()
	}
	flag.Parse()

	hosts := parseHosts(*hostFlag)
	if len(hosts) == 0 {
		fmt.Fprintln(os.Stderr, "server: -host is required")
		flag.Usage()
		os.Exit(2)
	}

	web, err := filepath.Abs(*webDir)
	if err != nil {
		log.Fatal(err)
	}
	info, err := os.Stat(web)
	if err != nil {
		log.Fatalf("web directory: %v", err)
	}
	if !info.IsDir() {
		log.Fatalf("web directory %s is not a directory", web)
	}

	cache, err := filepath.Abs(*cacheDir)
	if err != nil {
		log.Fatal(err)
	}

	manager := &autocert.Manager{
		Prompt:     autocert.AcceptTOS,
		Cache:      autocert.DirCache(cache),
		HostPolicy: autocert.HostWhitelist(hosts...),
		Email:      *email,
	}

	tlsConfig := manager.TLSConfig()
	tlsConfig.MinVersion = tls.VersionTLS12

	httpsServer := &http.Server{
		Addr:              *httpsAddr,
		Handler:           http.FileServer(http.Dir(web)),
		TLSConfig:         tlsConfig,
		ReadHeaderTimeout: 10 * time.Second,
	}
	httpServer := &http.Server{
		Addr:              *httpAddr,
		Handler:           manager.HTTPHandler(redirectToHTTPS(*httpsAddr)),
		ReadHeaderTimeout: 10 * time.Second,
	}

	log.Printf("serving %s for %s", web, strings.Join(hosts, ", "))
	log.Printf("https on %s, http on %s, certificate cache %s", *httpsAddr, *httpAddr, cache)

	errc := make(chan error, 2)
	go func() { errc <- httpServer.ListenAndServe() }()
	go func() { errc <- httpsServer.ListenAndServeTLS("", "") }()
	log.Fatal(<-errc)
}

func parseHosts(s string) []string {
	var hosts []string
	for _, part := range strings.Split(s, ",") {
		part = strings.TrimSpace(part)
		if part == "" {
			continue
		}
		if host, _, err := net.SplitHostPort(part); err == nil {
			part = host
		}
		hosts = append(hosts, part)
	}
	return hosts
}

func redirectToHTTPS(httpsAddr string) http.Handler {
	_, port, err := net.SplitHostPort(httpsAddr)
	if err != nil || port == "443" {
		port = ""
	}
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		host := r.Host
		if h, _, splitErr := net.SplitHostPort(r.Host); splitErr == nil {
			host = h
		}
		if port != "" {
			host = net.JoinHostPort(host, port)
		}
		target := url.URL{
			Scheme:   "https",
			Host:     host,
			Path:     r.URL.Path,
			RawQuery: r.URL.RawQuery,
		}
		http.Redirect(w, r, target.String(), http.StatusMovedPermanently)
	})
}
