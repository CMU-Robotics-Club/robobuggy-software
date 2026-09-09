#!/bin/bash
# Stop on the first failing command (a failed build must not proceed to `up`),
# treat unset variables as errors, and fail pipelines on any stage.
set -euo pipefail

dockerfile="docker-dev.yml"

echo "Stopping this project's development containers..."
# Only this compose project's services. The previous `docker stop $(docker ps -a -q)`
# stopped every container on the machine, including unrelated ones.
docker compose -f "$dockerfile" --env-file .env.dev down

echo "Building containers..."
docker compose -f "$dockerfile" --env-file .env.dev build

echo "Starting containers..."
docker compose -f "$dockerfile" --env-file .env.dev up -d

sleep 0.5

echo "DEBUG: Buggy Docker Container Up!"
echo "Run docker_exec in order to go into the Docker container"
