#!/bin/bash

pkg_path=$(ros2 pkg prefix kaiaai_python)
full_path="$pkg_path/share/kaiaai_python"
$full_path/ssl-proxy-linux-amd64 -from 0.0.0.0:4430 -to 127.0.0.1:8080 -redirectHTTP
