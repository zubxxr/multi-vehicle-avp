#!/bin/bash

# Define only the localization-relevant topics
TOPICS=(
  "/avp/vehicle_id"
  "/avp/vehicle_count"
  "/avp/parking_spots"
  "/avp/queue"
  "/avp/reserved_parking_spots"
  "/avp/status/all"
)

# Loop through each topic and echo it silently
for topic in "${TOPICS[@]}"
do
    echo "Echoing: $topic"
    ros2 topic echo "$topic" > /dev/null &
done

echo "All essential localization topics are being echoed in the background."

