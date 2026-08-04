#!/usr/bin/env bash
set -euo pipefail

repo=/home/wyk/teleoperated_driving
cd "$repo"

echo "== checking Logitech wheel in Ubuntu =="
lsusb | grep -Ei '046d|logitech|g923|g29|wheel' || true

wheel_js="$(
  awk '
    BEGIN { RS=""; FS="\n" }
    /Logitech|G923|G29|Racing|Wheel|Driving/ {
      for (i = 1; i <= NF; i++) {
        if ($i ~ /^N:/) print $i
        if ($i ~ /^H:/ && match($i, /js[0-9]+/)) print "/dev/input/" substr($i, RSTART, RLENGTH)
      }
    }
  ' /proc/bus/input/devices | tee /tmp/codex_g923_detect.txt | awk '/^\/dev\/input\/js[0-9]+$/ { print; exit }'
)"

if [[ -z "${wheel_js}" ]]; then
  echo "ERROR: Ubuntu VM does not see a Logitech G923/G29 joystick device yet."
  echo "In VMware, use: VM -> Removable Devices -> Logitech G923 -> Connect (Disconnect from host), then rerun this script."
  echo
  echo "Current joystick handlers:"
  awk 'BEGIN { RS=""; FS="\n" } /js[0-9]+/ { for (i = 1; i <= NF; i++) if ($i ~ /^N:|^H:/) print $i; print "" }' /proc/bus/input/devices
  exit 2
fi

echo "Using wheel device: ${wheel_js}"

if grep -q '^INPUT_DEVICE=' .env; then
  sed -i "s#^INPUT_DEVICE=.*#INPUT_DEVICE=${wheel_js}#" .env
else
  printf '\n# Host joystick device mapped to /dev/input/js0 in tod_operator\nINPUT_DEVICE=%s\n' "$wheel_js" >> .env
fi

cat > docker-compose.override.yaml <<'YAML'
services:
  g923_autocenter:
    image: ${DOCKER_REGISTRY}/tod_operator:${DOCKER_TAG:?}
    container_name: g923_autocenter_${DOCKER_TAG:?}
    user: "0:0"
    network_mode: none
    privileged: true
    volumes:
      - type: bind
        source: "/dev/input"
        target: /dev/input
        bind:
          create_host_path: false
      - type: bind
        source: "./work/set_g923_autocenter.py"
        target: /opt/tod-tools/set_g923_autocenter.py
        read_only: true
        bind:
          create_host_path: false
    environment:
      - G923_AUTOCENTER_STRENGTH=${G923_AUTOCENTER_STRENGTH:-80}
    command:
      - bash
      - -lc
      - >-
        python3 /opt/tod-tools/set_g923_autocenter.py
        "$${G923_AUTOCENTER_STRENGTH:-80}" ||
        { echo "WARNING: Autocenter unavailable; continuing"; exit 0; }
    restart: "no"

  tod_operator:
    depends_on:
      g923_autocenter:
        condition: service_completed_successfully
    volumes:
      - type: bind
        source: "/dev/input"
        target: /dev/input
        bind:
          create_host_path: false
      - type: bind
        source: "./work/logitechg923.yaml"
        target: /home/tum/wsp/install/tod_input_devices/share/tod_input_devices/config/virtual.yaml
        read_only: true
        bind:
          create_host_path: false
      - type: bind
        source: "./work/logitechg923.yaml"
        target: /home/tum/wsp/install/tod_input_devices/share/tod_input_devices/config/logitechg923.yaml
        read_only: true
        bind:
          create_host_path: false
YAML

if [[ ! -f work/logitechg923.yaml ]]; then
  echo "ERROR: Missing persistent G923 config: ${repo}/work/logitechg923.yaml"
  exit 3
fi

DISPLAY=:0 xhost +local:docker >/tmp/codex_xhost.log 2>&1 || true

docker compose down
docker compose up -d tod_vehicle tod_operator

echo "== waiting for persistent G923 input node =="
for i in $(seq 1 40); do
  publisher_count="$(
    docker exec tod_operator_latest bash -lc '
      source /opt/ros/humble/setup.bash
      source /home/tum/wsp/install/setup.bash
      ros2 topic info /operator/input_devices/output/joystick 2>/dev/null | awk "/Publisher count:/ {print \$3}"
    ' 2>/dev/null || true
  )"
  if [[ "$publisher_count" == "1" ]]; then
    break
  fi
  sleep 1
done

if [[ "${publisher_count:-0}" != "1" ]]; then
  echo "ERROR: G923 input publisher did not start."
  docker logs --tail 80 tod_operator_latest
  exit 4
fi

echo "== verifying persistent mounts =="
docker exec tod_operator_latest bash -lc '
  set -e
  test -c /dev/input/js0
  test -f /home/tum/wsp/install/tod_input_devices/share/tod_input_devices/config/virtual.yaml
  test -f /home/tum/wsp/install/tod_input_devices/share/tod_input_devices/config/logitechg923.yaml
  grep -q "Throttle: 2" /home/tum/wsp/install/tod_input_devices/share/tod_input_devices/config/virtual.yaml
  grep -q "Brake: 3" /home/tum/wsp/install/tod_input_devices/share/tod_input_devices/config/virtual.yaml
  ls -l /dev/input/js0 /home/tum/wsp/install/tod_input_devices/share/tod_input_devices/config/{virtual,logitechg923}.yaml
'

if [[ -f work/set_g923_autocenter.py ]]; then
  python3 work/set_g923_autocenter.py "${G923_AUTOCENTER_STRENGTH:-80}" || echo "WARNING: Could not apply G923 autocenter"
fi

echo "== current joystick topic sample =="
docker exec tod_operator_latest bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/tum/wsp/install/setup.bash
  timeout 8 ros2 topic echo --once /operator/input_devices/output/joystick || true
'

echo "== containers =="
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
