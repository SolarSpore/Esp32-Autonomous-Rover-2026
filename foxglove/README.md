# Foxglove

Foxglove dashboard layouts for the rover go here, as exported JSON files. None have been added yet.

## Setup

Installing Foxglove Studio and connecting it to the rover are covered in section 6 of
`../Setup-Guide.md`. The Rover Controls icon starts `foxglove_bridge`, which serves the rover's ROS
topics at `ws://localhost:8765`.

## Using a layout

Foxglove can export a layout as a JSON file and import it again. To use a layout from this folder,
import the file in Foxglove Studio. To add one, export it from Foxglove Studio and put the file in
this folder.
