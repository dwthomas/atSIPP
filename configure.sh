#!/bin/bash
meson setup --reconfigure build --clearcache
meson setup --reconfigure build_debug --clearcache
meson setup --reconfigure build_debugoptimized --clearcache
