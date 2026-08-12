#!/bin/bash
set -euo pipefail

rm /tmp/api_nvme.sock || true
rm /tmp/ch_serial.log || true

./target/release/cloud-hypervisor \
        --kernel ./CLOUDHV.fd \
        --disk path=oracular-server-cloudimg-amd64.raw,transport=Nvme \
        --cpus boot=4 \
        --memory size=4096M \
        --serial file=/tmp/ch_serial.log \
        --console off \
        --seccomp log \
        --api-socket /tmp/api_nvme.sock \
         -vv --log-file /tmp/ch_nvme.log
