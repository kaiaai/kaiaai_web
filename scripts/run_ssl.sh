#!/bin/bash

ssl-proxy-linux-amd64 -from 0.0.0.0:4430 -to 127.0.0.1:8080 -redirectHTTP
