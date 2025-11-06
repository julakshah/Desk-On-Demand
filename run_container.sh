#!/usr/bin/sh
docker run -it --rm $(for d in /dev/video*; do printf -- '--device=%s ' "$d"; done) \
    -e ROS_DOMAIN_ID=99 desk-on-demand bash
