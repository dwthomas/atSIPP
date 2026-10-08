#!/bin/bash
meson compile -C build &&
meson compile -C build_debug &&
meson compile -C build_debugoptimized
