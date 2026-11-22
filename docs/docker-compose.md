# Run with Docker Compose

These commands cover the same setup as the Makefile. Start Docker and run them from the repository root. Check that Docker Compose v2 is available.

```bash
docker compose version
```

## Build

On macOS, include the platform override.

```bash
docker compose -f compose.yaml -f compose.mac.yaml build web-sim web-mock
```

On Linux, use the base Compose file.

```bash
docker compose -f compose.yaml build web-sim web-mock
```

## Run the simulation

On macOS, start the simulator with the platform override.

```bash
docker compose -f compose.yaml -f compose.mac.yaml up web-sim
```

On Linux, start the simulator with the base Compose file.

```bash
docker compose -f compose.yaml up web-sim
```

Open the web app in your browser.

```bash
http://127.0.0.1:8001
```

### Optional MuJoCo desktop viewer

On macOS, start the simulator with the viewer enabled.

```bash
G1_SIM_VIEWER=1 docker compose -f compose.yaml -f compose.mac.yaml up web-sim
```

Open the noVNC viewer and click **Connect**.

```bash
http://127.0.0.1:6080/vnc.html
```

On Linux, use the X11 override with `DISPLAY` set and a readable Xauthority file at `$XAUTHORITY` or `~/.Xauthority`.

```bash
docker compose -f compose.yaml -f compose.x11.yaml up web-sim
```

## Run the mock

Use the same command on macOS and Linux.

```bash
docker compose -f compose.yaml up web-mock
```

Open the web app in your browser.

```bash
http://127.0.0.1:8000
```

## Open a shell for the SDK examples

Keep the simulator running and leave the web app in **Observation**. From a second terminal, open a shell in the container.

```bash
docker compose -f compose.yaml -f compose.mac.yaml exec web-sim bash  # macOS
docker compose -f compose.yaml exec web-sim bash                      # Linux
```

Run the [SDK examples](../README.md#run-the-sdk-examples) from that shell.

## Stop and remove the app

Press `Ctrl+C` to stop the foreground run. To remove the containers, run the command for your platform.

```bash
docker compose -f compose.yaml -f compose.mac.yaml --profile web-mock down  # macOS
docker compose -f compose.yaml --profile web-mock down                      # Linux
```
