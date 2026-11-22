#!/bin/sh
rm -rf web
mkdir web
GOOS=js GOARCH=wasm go build -o web/main.wasm *.go
GOOS=linux GOARCH=amd64 go build -o web/server server/main.go
cp $(go env GOROOT)/lib/wasm/wasm_exec.js web/
cp index.html web/
