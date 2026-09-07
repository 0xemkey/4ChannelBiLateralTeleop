#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
echo '[force-relay][blocked] C1/C4 external-torque qualification has not passed.' >&2
echo '[force-relay] Current approved setup keeps C1=0 and C4=0; no force-relay command is emitted.' >&2
exit 3

