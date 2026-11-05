# Toolchain for the AVR firmware and the firmware-in-the-loop harness:
#   docker build -t rtnav-dev docker
#   docker run --rm -v "$PWD":/src -w /src rtnav-dev make -C firmware
FROM ubuntu:24.04
RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      gcc-avr avr-libc binutils-avr simavr libsimavr-dev libelf-dev \
      cmake make gcc pkg-config python3 \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /src
