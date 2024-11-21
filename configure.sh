#!/bin/bash
meson setup --reconfigure build
meson setup --reconfigure build_debug
meson setup --reconfigure build_debugoptimized
