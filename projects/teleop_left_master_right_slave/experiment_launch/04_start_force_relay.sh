#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); ARM_RELAY=false; CHANNEL_MODE=""; CHANNEL_MODE_COUNT=0
source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
for arg in "${EXTRA_ARGS[@]}"; do
  case "${arg}" in
    --arm-relay) ARM_RELAY=true ;;
    --c1-only) CHANNEL_MODE="c1-only"; CHANNEL_MODE_COUNT=$((CHANNEL_MODE_COUNT + 1)) ;;
    --c4-only) CHANNEL_MODE="c4-only"; CHANNEL_MODE_COUNT=$((CHANNEL_MODE_COUNT + 1)) ;;
    --full4ch) CHANNEL_MODE="full4ch"; CHANNEL_MODE_COUNT=$((CHANNEL_MODE_COUNT + 1)) ;;
    *) echo "[force-relay][error] unknown argument: ${arg}" >&2; exit 2 ;;
  esac
done

[[ "${CHANNEL_MODE_COUNT}" -eq 1 ]] || {
  echo "[force-relay][error] choose exactly one: --c1-only | --c4-only | --full4ch" >&2
  exit 2
}

reject_live_component 'bilateral_4ch.py' 'bilateral relay'

case "${CHANNEL_MODE}" in
  c1-only)
    W1="${C1_WEIGHT:-1.0}"; W2=0.0; W3=0.0; W4=0.0
    ;;
  c4-only)
    W1=0.0; W2=0.0; W3=0.0; W4="${C4_WEIGHT:-1.0}"
    ;;
  full4ch)
    # Position channels are owned by bridge_master_slave.py in this hardware
    # architecture. Keep bilateral_4ch.py's joint-space setpoint blend at zero
    # to avoid duplicate position authority and misleading active flags.
    W1="${C1_WEIGHT:-0.5}"; W2=0.0; W3=0.0; W4="${C4_WEIGHT:-0.3}"
    ;;
esac

C1_DISABLE_JOINTS="${C1_DISABLE_JOINTS:-}"
CF="${CF:-1.0}"

cmd=(python3 "${PROJECT_ROOT}/scripts/bilateral_4ch.py"
  --architecture custom
  --w1 "${W1}" --w2 "${W2}" --w3 "${W3}" --w4 "${W4}"
  --cf "${CF}"
  --c1-disable-joints "${C1_DISABLE_JOINTS}"
)

echo "[force-relay] mode=${CHANNEL_MODE} weights(C1..C4)=${W1},${W2},${W3},${W4} cf=${CF}"
echo "[force-relay] C1 disabled joints='${C1_DISABLE_JOINTS}' (empty means no joint masking)"
echo '[force-relay][warning] this node must NOT publish /enable in the supervised bridge architecture.'
echo '[force-relay][warning] applied_force_n on CartesianControllerTelemetry remains the device-write ground truth.'
echo "[force-relay] emergency stop: ${LAUNCH_DIR}/99_stop_setup.sh --session ${SESSION_ID} --output-dir ${OUTPUT_DIR}"
if [[ "${DRY_RUN}" == false && "${ARM_RELAY}" != true ]]; then
  echo '[force-relay][blocked] operator confirmation flag --arm-relay is required' >&2
  exit 3
fi
if [[ "${DRY_RUN}" == false ]]; then
  require_session
  {
    printf 'FORCE_RELAY_MODE=%q\n' "${CHANNEL_MODE}"
    printf 'FORCE_RELAY_WEIGHTS_C1_C2_C3_C4=%q\n' "${W1},${W2},${W3},${W4}"
    printf 'FORCE_RELAY_CF=%q\n' "${CF}"
    printf 'FORCE_RELAY_C1_DISABLE_JOINTS=%q\n' "${C1_DISABLE_JOINTS}"
    printf 'FORCE_RELAY_PUBLISH_ENABLE=%q\n' 'false'
  } >>"${MANIFEST}"
  source_ros
fi
start_component force_relay "${cmd[@]}"
