#!/bin/bash
# Run the grading environment: mount repo at /source, no hardware device.
# For real vehicle/serial hardware, add: --device /dev/ttyUSB0:/dev/ttyUSB0
docker run --rm -v $PWD:/source -it klavins/elma:latest bash
