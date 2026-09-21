#!/bin/bash
meson setup --reconfigure build --clearcache \
  -Dprefer_static=true -Ddefault_library=static \
  -Dc_link_args=-static -Dcpp_link_args=-static

meson setup --reconfigure build_debug --clearcache \
  -Dprefer_static=true -Ddefault_library=static \
  -Dc_link_args=-static -Dcpp_link_args=-static

meson setup --reconfigure build_debugoptimized --clearcache \
  -Dprefer_static=true -Ddefault_library=static \
  -Dc_link_args=-static -Dcpp_link_args=-static
