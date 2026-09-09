# Development network exposure

`docker-dev.yml` publishes three ports: 8765 (Foxglove bridge, WebSocket) and
8760 (asset server for Foxglove) from the `main` container, and 8080 (map tile
server) from `tileserver`.

## Default: localhost only

All three bind to `127.0.0.1` by default, so only clients on the same machine
(a local Foxglove Studio, a browser on this laptop) can reach them. Other hosts
on the network cannot connect even if the laptop's firewall would allow it.

The Foxglove bridge has **no authentication**: anyone who can open the WebSocket
can subscribe to every topic and, depending on the bridge configuration, publish
and call services. Exposing it beyond localhost should be a deliberate choice.

## Exposing on the LAN deliberately

To let a remote Foxglove client (another laptop at the track, a phone hotspot)
connect, set `BIND_ADDR` before bringing the stack up, either in `.env.dev`:

    BIND_ADDR=0.0.0.0

or in the shell for a single run:

    BIND_ADDR=0.0.0.0 docker compose -f docker-dev.yml --env-file .env.dev up -d

`BIND_ADDR` is read by `docker compose` when it renders the file (from
`--env-file` or the shell), not by anything inside the container. Restart the
stack after changing it and switch back to the default when you are done.

## setup_dev.sh scope

`setup_dev.sh` now stops and restarts only this project's compose services
(`docker compose ... down`). It used to run `docker stop $(docker ps -a -q)`,
which stopped every container on the machine. It also aborts on the first
failed step, so a broken build no longer proceeds to `up`.
